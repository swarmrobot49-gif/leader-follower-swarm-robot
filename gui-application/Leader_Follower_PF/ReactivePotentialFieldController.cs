using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Globalization;



namespace HybridDijkstraPotentialField
{
    /// <summary>
    /// Pure Reactive Potential Field Controller untuk Leader Robot
    /// VERSI 2: Menggunakan callback untuk kirim command, tidak buat koneksi TCP sendiri
    /// </summary>
    public class ReactivePotentialFieldController : IDisposable
    {
        // ============================================================
        // TUNABLE PARAMETERS
        // ============================================================

        public float AttractiveGain { get; set; } = 7.0f;
        public float AttractiveMaxForce { get; set; } = 30.0f;

        public float RepulsiveGain { get; set; } = 1000.0f;
        public float ObstacleInfluenceRange { get; set; } = 50.0f;
        public float RepulsiveMaxForce { get; set; } = 40.0f;

        public float MaxLinearVelocity { get; set; } = 8.0f;
        public float MaxAngularVelocity { get; set; } = 1.3f;
        public float MinLinearVelocity { get; set; } = 3.0f;

        public float GoalReachedThreshold { get; set; } = 6.0f;
        public float TurnInPlaceThreshold { get; set; } = 1.0f;
        public float EmergencyStopDistance { get; set; } = 10.0f;

        public float VelocityLowPassAlpha { get; set; } = 0.3f;
        public float ForceLowPassAlpha { get; set; } = 0.2f;

        public float VortexGain { get; set; } = 0.5f; // 0 = off, coba 0.3–0.7
        public float ObstacleSlowdownMinScale { get; set; } = 0.45f; // v minimal saat sangat dekat obstacle
        public float AvoidTurnMaxAngleRad { get; set; } = 0.9f; // 45 deg
        public float AvoidDistance { get; set; } = 80.0f;         // cm: mulai mode avoid
        public float HeadingLowPassAlpha { get; set; } = 0.4f;   // smoothing arah force

        public float WallFollowGain { get; set; } = 0.7f;
        public int StuckCounterThreshold { get; set; } = 30;

        // ============================================================
        // STATE
        // ============================================================

        private float robotX, robotY, robotTheta;
        private float goalX, goalY;
        private bool hasRobotPose = false;
        private bool hasGoal = false;
        private bool goalReached = false;

        private List<PointF> obstacles = new List<PointF>();
        private readonly object obstaclesLock = new object();

        private float filteredDesiredTheta = 0f;
        private bool hasFilteredDesiredTheta = false;

        private float filteredV = 0, filteredW = 0;
        private float filteredForceX = 0, filteredForceY = 0;
        private float lastTotalForceX = 0, lastTotalForceY = 0;

        private float lastDistanceToGoal = float.MaxValue;
        private int stuckCounter = 0;
        private bool isWallFollowing = false;

        private static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        private List<PointF> clusteredObstacles = new List<PointF>();
        private float ObstacleClusterRadius { get; set; } = 20.0f;

        private float arenaWidth = 200f;
        private float arenaHeight = 150f;

        // After line ~60
        private float lastMinObstDist = float.MaxValue;
        private const float PENETRATION_THRESHOLD = 5.0f; // cm

        // Control loop
        private bool isRunning = false;
        private CancellationTokenSource controlCts;
        private Task controlTask;
        private int controlLoopIntervalMs = 50;

        // ============================================================
        private Random randomWalk = new Random();
        private bool isRandomWalkActive = false;
        private int randomWalkCountdown = 0;
        private float randomWalkTargetAngle = 0f;
        private int consecutiveLowForceCount = 0;
        private const int LOW_FORCE_THRESHOLD_COUNT = 10;
        const float LocalMinimumForceThreshold = 5.0f;

        // ============================================================
        // CALLBACK UNTUK KIRIM COMMAND (tidak pakai TCP sendiri)
        // ============================================================

        /// <summary>
        /// Callback untuk mengirim command ke robot.
        /// Format: "SETVEL v w" atau "STOP"
        /// </summary>
        public Action<string> SendCommandCallback { get; set; }

        // ============================================================
        // EVENTS
        // ============================================================

        public event Action<string> OnStatusChanged;
        public event Action<float, float, float, float> OnVelocityCommand; // v, w, forceX, forceY
        public event Action OnGoalReached;
        public event Action<string> OnDebugMessage;

        // ============================================================
        // PUBLIC PROPERTIES
        // ============================================================

        public bool IsRunning => isRunning;
        public bool HasGoal => hasGoal;
        public bool IsWallFollowing => isWallFollowing;

        // ============================================================
        // CONSTRUCTOR
        // ============================================================

        public ReactivePotentialFieldController()
        {
            // Tidak perlu parameter koneksi - pakai callback
        }

        // ============================================================
        // SETUP
        // ============================================================

        public void SetArenaSize(float width, float height)
        {
            arenaWidth = width;
            arenaHeight = height;
        }

        public void SetGoal(float x, float y)
        {
            goalX = x;
            goalY = y;
            hasGoal = true;
            goalReached = false;
            stuckCounter = 0;
            isWallFollowing = false;

            OnDebugMessage?.Invoke($"Goal set: ({x:F1}, {y:F1}) cm");
            OnStatusChanged?.Invoke($"Goal: ({x:F0}, {y:F0}) cm");
        }

        public void SetGoalFromGrid(int gridX, int gridY, float cellSizeCm)
        {
            float x = gridX * cellSizeCm + cellSizeCm / 2.0f;
            float y = gridY * cellSizeCm + cellSizeCm / 2.0f;
            SetGoal(x, y);
        }

        // ============================================================
        // POSE & OBSTACLE UPDATE
        // ============================================================

        public void UpdateLeaderPose(float x, float y, float theta)
        {
            robotX = x;
            robotY = y;
            robotTheta = theta;

            bool wasHavePose = hasRobotPose;
            hasRobotPose = true;

            if (!wasHavePose)
            {
                OnDebugMessage?.Invoke($"FIRST POSE: ({x:F1}, {y:F1}, {theta:F2} rad)");
            }
        }

        public void UpdateObstacles(List<PointF> newObstacles)
        {
            lock (obstaclesLock)
            {
                obstacles.Clear();
                if (newObstacles != null)
                {
                    obstacles.AddRange(newObstacles);
                }
            }
        }

        // ============================================================
        // GETTERS
        // ============================================================

        public PointF GetGoalPosition() => new PointF(goalX, goalY);
        public PointF GetRobotPosition() => new PointF(robotX, robotY);
        public PointF GetLastTotalForce() => new PointF(lastTotalForceX, lastTotalForceY);

        public float GetDistanceToGoal()
        {
            if (!hasGoal) return float.MaxValue;
            float dx = goalX - robotX;
            float dy = goalY - robotY;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        public List<PointF> GetObstacles()
        {
            lock (obstaclesLock)
            {
                return new List<PointF>(obstacles);
            }
        }
        public List<PointF> GetClusteredObstacles()
        {
            return new List<PointF>(clusteredObstacles);
        }

        // ============================================================
        // CONTROL START/STOP
        // ============================================================

        public void StartControl()
        {
            if (isRunning) return;

            if (SendCommandCallback == null)
            {
                OnDebugMessage?.Invoke("ERROR: SendCommandCallback not set!");
                OnStatusChanged?.Invoke("ERROR: No command callback!");
                return;
            }

            isRunning = true;
            controlCts = new CancellationTokenSource();

            OnDebugMessage?.Invoke($"StartControl: hasRobotPose={hasRobotPose}, hasGoal={hasGoal}");
            OnDebugMessage?.Invoke($"Robot: ({robotX:F1}, {robotY:F1}), Goal: ({goalX:F1}, {goalY:F1})");

            controlTask = Task.Run(async () =>
            {
                OnStatusChanged?.Invoke("Reactive PF RUNNING");

                int loopCount = 0;

                while (!controlCts.Token.IsCancellationRequested)
                {
                    try
                    {
                        loopCount++;

                        if (hasRobotPose && hasGoal && !goalReached)
                        {
                            CalculateControlOutput(out float v, out float w);
                            SendVelocityCommand(v, w);

                            // Debug setiap 20 loop (1 detik)
                            if (loopCount % 20 == 0)
                            {
                                float dist = GetDistanceToGoal();
                                OnDebugMessage?.Invoke($"v={v:F1} w={w:F2} dist={dist:F1}cm");
                            }
                        }
                        else if (!hasRobotPose)
                        {
                            if (loopCount % 40 == 0)
                            {
                                OnDebugMessage?.Invoke("WARNING: No robot pose!");
                                OnStatusChanged?.Invoke("Waiting for pose...");
                            }
                        }
                        else if (goalReached)
                        {
                            SendStopCommand();
                        }

                        await Task.Delay(controlLoopIntervalMs, controlCts.Token);
                    }
                    catch (OperationCanceledException)
                    {
                        break;
                    }
                    catch (Exception ex)
                    {
                        OnDebugMessage?.Invoke($"Loop error: {ex.Message}");
                    }
                }

                SendStopCommand();
                OnStatusChanged?.Invoke("Reactive PF STOPPED");
            }, controlCts.Token);
        }

        public void StopControl()
        {
            if (!isRunning) return;

            isRunning = false;
            controlCts?.Cancel();

            try
            {
                controlTask?.Wait(500);
            }
            catch { }

            SendStopCommand();

            filteredV = 0;
            filteredW = 0;
        }

        private List<PointF> ClusterObstacles(List<PointF> rawObstacles)
        {
            // Jika tidak ada obstacle, return empty list
            if (rawObstacles == null || rawObstacles.Count == 0)
                return new List<PointF>();

            // Jika hanya ada sedikit obstacle, tidak perlu clustering
            if (rawObstacles.Count <= 3)
                return new List<PointF>(rawObstacles);

            // Simple clustering: merge obstacles yang sangat dekat
            const float CLUSTER_THRESHOLD = 15.0f; // 15cm

            List<PointF> clustered = new List<PointF>();
            bool[] used = new bool[rawObstacles.Count];

            for (int i = 0; i < rawObstacles.Count; i++)
            {
                if (used[i]) continue;

                // Start new cluster
                float sumX = rawObstacles[i].X;
                float sumY = rawObstacles[i].Y;
                int count = 1;
                used[i] = true;

                // Find nearby obstacles to merge
                for (int j = i + 1; j < rawObstacles.Count; j++)
                {
                    if (used[j]) continue;

                    float dx = rawObstacles[i].X - rawObstacles[j].X;
                    float dy = rawObstacles[i].Y - rawObstacles[j].Y;
                    float dist = (float)Math.Sqrt(dx * dx + dy * dy);

                    if (dist < CLUSTER_THRESHOLD)
                    {
                        sumX += rawObstacles[j].X;
                        sumY += rawObstacles[j].Y;
                        count++;
                        used[j] = true;
                    }
                }

                // Add cluster centroid
                clustered.Add(new PointF(sumX / count, sumY / count));
            }

            return clustered;
        }

        // ============================================================
        // FORCE CALCULATION
        // ============================================================

        private void CalculateControlOutput(out float v, out float w)
        {
            // ===== DECLARE ALL VARIABLES AT THE TOP (SCOPE FIX) =====
            float headingError = 0f;
            float rawW = 0f;
            float rawV = 0f;

            // 1. Calculate attractive force
            float dx = goalX - robotX;
            float dy = goalY - robotY;
            float distToGoal = (float)Math.Sqrt(dx * dx + dy * dy);

            // Check goal reached
            if (distToGoal < GoalReachedThreshold)
            {
                v = 0;
                w = 0;
                goalReached = true;
                isRandomWalkActive = false; // Reset random walk
                consecutiveLowForceCount = 0;
                OnGoalReached?.Invoke();
                OnStatusChanged?.Invoke("GOAL REACHED!");
                return;
            }

            // ===== CALCULATE FORCES =====
            float repForceX = 0, repForceY = 0;
            float minObstDist = float.MaxValue;
            float closestOdx = 0f, closestOdy = 0f;

            // CLUSTER obstacles terlebih dahulu
            List<PointF> activeObstacles;
            lock (obstaclesLock)
            {
                activeObstacles = ClusterObstacles(obstacles);
                clusteredObstacles = activeObstacles; // Cache for visualization
            }

            // Calculate repulsive forces
            foreach (var obs in activeObstacles)
            {
                float odx = robotX - obs.X;
                float ody = robotY - obs.Y;
                float dist = (float)Math.Sqrt(odx * odx + ody * ody);

                if (lastMinObstDist != float.MaxValue && minObstDist < lastMinObstDist - 3.0f)
                {
                    // Obstacle distance decreasing rapidly! Possible collision
                    OnDebugMessage?.Invoke($"⚠️ RAPID APPROACH! {lastMinObstDist:F1}→{minObstDist:F1}cm");

                    // Emergency brake with higher force
                    if (minObstDist < PENETRATION_THRESHOLD)
                    {
                        filteredV = 0;
                        filteredW = 0;
                        v = 0;
                        w = 0;
                        OnStatusChanged?.Invoke($"COLLISION IMMINENT! Stopping at {minObstDist:F1}cm");
                        return;
                    }
                }
                lastMinObstDist = minObstDist;

                if (dist < minObstDist)
                {
                    minObstDist = dist;
                    closestOdx = odx;
                    closestOdy = ody;
                }

                if (dist < ObstacleInfluenceRange && dist > 0.1f)
                {
                    float repMag = RepulsiveGain * (1.0f / dist - 1.0f / ObstacleInfluenceRange) / (dist * dist);
                    repMag = Math.Min(repMag, RepulsiveMaxForce);

                    repForceX += repMag * (odx / dist);
                    repForceY += repMag * (ody / dist);
                }
            }

            // Attractive force (linear, capped)
            float attMag = AttractiveGain * distToGoal;
            if (attMag > AttractiveMaxForce) attMag = AttractiveMaxForce;

            float attForceX = attMag * (dx / distToGoal);
            float attForceY = attMag * (dy / distToGoal);

            // Vortex / tangential force
            if (VortexGain > 0f && minObstDist < ObstacleInfluenceRange && minObstDist > 0.1f)
            {
                float inv = 1f / minObstDist;

                // arah radial menjauhi obstacle (unit)
                float rx = closestOdx * inv;
                float ry = closestOdy * inv;

                // dua kandidat tangensial (kiri/kanan)
                float tx1 = -ry, ty1 = rx;
                float tx2 = ry, ty2 = -rx;

                // pilih tangensial yang lebih "searah goal"
                float ginv = 1f / Math.Max(distToGoal, 1e-3f);
                float gx = dx * ginv, gy = dy * ginv;

                float dot1 = tx1 * gx + ty1 * gy;
                float dot2 = tx2 * gx + ty2 * gy;

                float tx = (dot1 >= dot2) ? tx1 : tx2;
                float ty = (dot1 >= dot2) ? ty1 : ty2;

                // besarnya vortex naik saat makin dekat obstacle
                float s = 1f - (minObstDist / ObstacleInfluenceRange);
                float vortexMag = VortexGain * RepulsiveMaxForce * Clamp01(s);

                repForceX += vortexMag * tx;
                repForceY += vortexMag * ty;
            }

            // 3. Emergency stop
            if (minObstDist < EmergencyStopDistance)
            {
                filteredV = 0;     // ← ADD THIS
                filteredW = 0;
                v = 0;
                w = 0;
                isRandomWalkActive = false;
                consecutiveLowForceCount = 0;
                OnStatusChanged?.Invoke("EMERGENCY STOP!");
                return;
            }

            // 4. Total force
            float totalForceX = attForceX + repForceX;
            float totalForceY = attForceY + repForceY;

            // Low-pass filter
            filteredForceX += ForceLowPassAlpha * (totalForceX - filteredForceX);
            filteredForceY += ForceLowPassAlpha * (totalForceY - filteredForceY);

            lastTotalForceX = filteredForceX;
            lastTotalForceY = filteredForceY;

            // ===== LOCAL MINIMUM DETECTION =====
            float forceMag = (float)Math.Sqrt(filteredForceX * filteredForceX + filteredForceY * filteredForceY);

            // Detect local minimum: weak force tapi masih jauh dari goal
            bool isInLocalMinimum = (distToGoal > 15.0f && forceMag < 5.0f && minObstDist > EmergencyStopDistance);

            if (isInLocalMinimum)
            {
                consecutiveLowForceCount++;

                // Activate random walk after 0.75 seconds of low force
                if (consecutiveLowForceCount >= LOW_FORCE_THRESHOLD_COUNT && !isRandomWalkActive)
                {
                    isRandomWalkActive = true;
                    randomWalkCountdown = 60; // 3 seconds @ 20Hz

                    // Generate random angle with bias towards goal
                    float goalAngle = (float)Math.Atan2(dy, dx);
                    float randomOffset = (float)(randomWalk.NextDouble() * Math.PI - Math.PI / 2); // ±90°
                    randomWalkTargetAngle = WrapAngle(goalAngle + randomOffset);

                    OnDebugMessage?.Invoke($"⚠️ LOCAL MINIMUM DETECTED! force={forceMag:F2}, dist={distToGoal:F1}cm");
                    OnDebugMessage?.Invoke($"   Activating random walk towards {randomWalkTargetAngle * 180 / Math.PI:F0}°");
                    OnStatusChanged?.Invoke($"Escaping local minimum... ({consecutiveLowForceCount})");
                }
            }
            else
            {
                consecutiveLowForceCount = 0;
            }

            // ===== RANDOM WALK EXECUTION =====
            if (isRandomWalkActive && randomWalkCountdown > 0)
            {
                randomWalkCountdown--;

                // Calculate heading error to random target (REUSE VARIABLE)
                headingError = WrapAngle(randomWalkTargetAngle - robotTheta);

                // ESCAPE BEHAVIOR: Move with bias toward random direction
                rawW = 2.5f * headingError; // Slightly faster turning
                rawW = Math.Max(-MaxAngularVelocity, Math.Min(MaxAngularVelocity, rawW));

                if (Math.Abs(headingError) > 0.5f) // ~30°
                {
                    // Turn in place if heading error is large
                    rawV = 0;
                }
                else
                {
                    // Move forward at moderate speed
                    rawV = MaxLinearVelocity * 0.6f;

                    // Reduce speed if very close to obstacle
                    if (minObstDist < ObstacleInfluenceRange * 0.5f)
                    {
                        rawV *= (minObstDist / (ObstacleInfluenceRange * 0.5f));
                    }
                }

                // Apply velocity filtering
                filteredV += VelocityLowPassAlpha * (rawV - filteredV);
                filteredW += VelocityLowPassAlpha * (rawW - filteredW);

                v = filteredV;
                w = filteredW;

                // Check if random walk completed
                if (randomWalkCountdown == 0)
                {
                    isRandomWalkActive = false;
                    consecutiveLowForceCount = 0;
                    OnDebugMessage?.Invoke("✅ Random walk complete, resuming normal PF control");
                    OnStatusChanged?.Invoke("Resuming normal navigation");
                }

                // Fire event
                OnVelocityCommand?.Invoke(v, w, lastTotalForceX, lastTotalForceY);
                return; // Skip normal PF calculation
            }

            // ===== NORMAL PF CONTROL (if not in random walk) =====

            // 5. Stuck detection (existing logic)
            if (Math.Abs(distToGoal - lastDistanceToGoal) < 0.5f)
            {
                stuckCounter++;
                if (stuckCounter > StuckCounterThreshold)
                {
                    isWallFollowing = true;
                    OnDebugMessage?.Invoke($"Wall-following activated (stuck count: {stuckCounter})");
                }
            }
            else
            {
                stuckCounter = 0;
                isWallFollowing = false;
            }
            lastDistanceToGoal = distToGoal;

            // 6. Wall following (rotate force 90°) - ENHANCED VERSION
            

            // 7. Convert force to velocity
            forceMag = (float)Math.Sqrt(filteredForceX * filteredForceX + filteredForceY * filteredForceY);

            if (forceMag < 0.1f)
            {
                v = 0;
                w = 0;
                return;
            }

            // === SAFETY ZONE: Hard limit velocity near obstacles ===
            if (minObstDist < EmergencyStopDistance * 1.5f) // 27cm safety zone
            {
                float safetyScale = (minObstDist - EmergencyStopDistance) /
                                    (EmergencyStopDistance * 0.5f);
                safetyScale = Math.Max(0.0f, Math.Min(1.0f, safetyScale));

                // Hard cap linear velocity
                rawV = Math.Min(rawV, MaxLinearVelocity * 0.5f * safetyScale);

                OnDebugMessage?.Invoke($"SAFETY ZONE: dist={minObstDist:F1}cm, scale={safetyScale:F2}");
            }

            // Desired heading from force
            float desiredTheta = (float)Math.Atan2(filteredForceY, filteredForceX);

            // Smooth desired heading supaya tidak "loncat-loncat"
            if (!hasFilteredDesiredTheta)
            {
                filteredDesiredTheta = desiredTheta;
                hasFilteredDesiredTheta = true;
            }
            else
            {
                filteredDesiredTheta = LerpAngle(filteredDesiredTheta, desiredTheta, HeadingLowPassAlpha);
            }
            desiredTheta = filteredDesiredTheta;

            // Heading error (REUSE VARIABLE)
            headingError = WrapAngle(desiredTheta - robotTheta);

            // Saat dekat obstacle: batasi arah belok max ±45° (tidak akan minta putar 180°)
            // MODIFIED: Don't limit if in wall-following mode
            bool nearObstacle = (minObstDist < AvoidDistance);
            if (nearObstacle && !isWallFollowing)
            {
                headingError = Clamp(headingError, -AvoidTurnMaxAngleRad, AvoidTurnMaxAngleRad);
            }

            // Angular velocity (REUSE VARIABLE)
            rawW = 2.0f * headingError;
            rawW = Math.Max(-MaxAngularVelocity, Math.Min(MaxAngularVelocity, rawW));

            // Linear velocity (reduce when turning) (REUSE VARIABLE)
            rawV = MaxLinearVelocity;

            if (minObstDist < ObstacleInfluenceRange)
            {
                float denom = Math.Max(1e-3f, (ObstacleInfluenceRange - EmergencyStopDistance));
                float t = (minObstDist - EmergencyStopDistance) / denom;   // 0..1
                t = Math.Max(0.0f, Math.Min(1.0f, t));

                // Kurva lebih "ramah": sqrt -> tidak terlalu ngerem saat masih cukup jauh
                t = (float)Math.Sqrt(t);

                // Kalau heading sudah bagus (belok kecil), jangan direm banyak
                float turn = Math.Abs(headingError) / Math.Max(1e-3f, TurnInPlaceThreshold);
                turn = Math.Max(0.0f, Math.Min(1.0f, turn));

                // Blend: saat turn=0 => scale ~1, saat turn=1 => scale ~t
                float scale = (1.0f - turn) * 1.0f + turn * t;

                // Clamp minimum
                scale = Math.Max(ObstacleSlowdownMinScale, Math.Min(1.0f, scale));
                rawV *= scale;
            }

            // ========== NORMAL CONTROL ==========

            // Turn in place logic (with RELAXED threshold)
            if (Math.Abs(headingError) > TurnInPlaceThreshold) // Now 1.2 rad (68.8°)
            {
                rawV = 0; // Turn in place
            }
            else
            {
                float turnFactor = 1.0f - Math.Abs(headingError) / (float)Math.PI;
                rawV *= turnFactor;
            }

            // Slow down near goal (LESS AGGRESSIVE)
            if (distToGoal < 30.0f)  // Changed from 30cm to 20cm
            {
                float slowdownFactor = distToGoal / 20.0f;
                slowdownFactor = Math.Max(0.3f, slowdownFactor); // Minimum 40% speed (not 17%)
                rawV *= slowdownFactor;
            }

            // ENFORCE minimum velocity when approaching goal
            if (distToGoal < 15.0f && rawV > 0.1f && rawV < 3.5f)
            {
                rawV = 3.5f; // Force minimum 3.5 cm/s for final approach
            }

            // Minimum velocity (general)
            if (rawV > 0.1f && rawV < MinLinearVelocity)
            {
                rawV = MinLinearVelocity;
            }

            

            // Low-pass filter velocity
            filteredV += VelocityLowPassAlpha * (rawV - filteredV);
            filteredW += VelocityLowPassAlpha * (rawW - filteredW);

            v = filteredV;
            w = filteredW;

            // Fire event
            OnVelocityCommand?.Invoke(v, w, lastTotalForceX, lastTotalForceY);
        }

        // ============================================================
        // SEND COMMANDS (via callback)
        // ============================================================

        private void SendVelocityCommand(float v, float w)
        {
            if (SendCommandCallback == null) return;

            string cmd = $"SETVEL {v.ToString("F2", CI)} {w.ToString("F3", CI)}";


            try
            {
                SendCommandCallback(cmd);
            }
            catch (Exception ex)
            {
                OnDebugMessage?.Invoke($"Send error: {ex.Message}");
            }
        }

        private void SendStopCommand()
        {
            if (SendCommandCallback == null) return;

            try
            {
                SendCommandCallback("SETVEL 0 0");
            }
            catch { }
        }

        

        // ============================================================
        // DISPOSE
        // ============================================================
        private static float Clamp(float v, float lo, float hi) => Math.Max(lo, Math.Min(hi, v));

        private static float Clamp01(float v) => Clamp(v, 0f, 1f);

        private static float WrapAngle(float a)
        {
            while (a > Math.PI) a -= 2f * (float)Math.PI;
            while (a < -Math.PI) a += 2f * (float)Math.PI;
            return a;
        }

        // Lerp sudut dengan mempertimbangkan wrap (-pi..pi)
        private static float LerpAngle(float a, float b, float t)
        {
            float d = WrapAngle(b - a);
            return WrapAngle(a + d * t);
        }

        public void Dispose()
        {
            StopControl();
            controlCts?.Dispose();
        }
    }
}