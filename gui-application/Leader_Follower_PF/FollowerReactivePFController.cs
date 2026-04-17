using System;
using System.Collections.Generic;
using System.Drawing;
using System.Net.Sockets;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Globalization;

namespace HybridDijkstraPotentialField
{
    /// <summary>
    /// Follower Reactive Potential Field Controller
    /// Follower mengikuti Leader dengan reactive avoidance terhadap obstacles
    /// Goal = posisi Leader (dynamic goal)
    /// </summary>
    public class FollowerReactivePFController : IDisposable
    {
        // ============================================================
        // TUNING PARAMETERS
        // ============================================================

        // Following Parameters
        public float FollowDistance { get; set; } = 25.0f;          // cm - jarak ideal di belakang leader
        public float StopDistance { get; set; } = 20.0f;            // cm - berhenti jika terlalu dekat
        public float ResumeDistance { get; set; } = 30.0f;          // cm - mulai gerak lagi jika leader menjauh

        // Attractive Force (ke anchor point di belakang leadfer)
        public float AttractiveGain { get; set; } = 7.0f;
        public float AttractiveMaxForce { get; set; } = 10.0f;

        // Repulsive Force (dari obstacles)
        public float RepulsiveGain { get; set; } = 300.0f;
        public float ObstacleInfluenceRange { get; set; } = 10.0f;  // cm
        public float RepulsiveMaxForce { get; set; } = 25.0f;

        // Leader Avoidance (treat leader as special obstacle)
        public float LeaderAvoidanceGain { get; set; } = 900.0f;
        public float LeaderAvoidanceRange { get; set; } = 38.0f;

        // Velocity Limits
        public float MaxLinearVelocity { get; set; } = 10.0f;       // cm/s
        public float MaxAngularVelocity { get; set; } = 1.3f;       // rad/s
        public float MinLinearVelocity { get; set; } = 3.0f;        // cm/s

        // Control Parameters
        public float AngleDeadband { get; set; } = 0.5f;           // rad
        public float TurnInPlaceThreshold { get; set; } = 0.9f;     // rad


        // Smoothing
        public float VelocityLowPassAlpha { get; set; } = 0.5f;
        public float AnchorLowPassAlpha { get; set; } = 0.2f;       // Smooth anchor position

        // Safety
        public float EmergencyStopDistance { get; set; } = 10.0f;   // cm

        // ============================================================
        // STATE VARIABLES
        // ============================================================

        // Follower State
        private float followerX = 0, followerY = 0, followerTheta = 0;
        private bool hasFollowerPose = false;

        // Leader State (dynamic goal)
        private float leaderX = 0, leaderY = 0, leaderTheta = 0;
        private bool hasLeaderPose = false;
        private DateTime lastLeaderUpdate = DateTime.MinValue;
        private readonly TimeSpan leaderTimeout = TimeSpan.FromMilliseconds(500);

        // Anchor Point (filtered target behind leader)
        private float anchorX = 0, anchorY = 0;
        private bool anchorInitialized = false;

        // Obstacles
        private List<PointF> obstacles = new List<PointF>();
        private readonly object obstacleLock = new object();

        // Filtered Values
        private float filteredLinearVel = 0, filteredAngularVel = 0;

        // State Machine
        private enum FollowerState { Idle, Following, Stopped, Catching }
        private FollowerState currentState = FollowerState.Idle;



        // Control Loop
        private CancellationTokenSource controlCts;
        private Task controlTask;
        private bool isRunning = false;
        private readonly int controlLoopIntervalMs = 50; // 20 Hz

        // Network
        private TcpClient followerClient;
        private StreamWriter followerWriter;
        private string followerIP;
        private int followerPort;

        // Events
        public event Action<string> OnStatusChanged;
        public event Action<float, float> OnVelocityCommand;
        public event Action<string> OnDebugMessage;

        private static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        // ============================================================
        // CONSTRUCTOR
        // ============================================================

        public FollowerReactivePFController(string followerIP = "192.168.137.155", int followerPort = 6060)
        {
            this.followerIP = followerIP;
            this.followerPort = followerPort;
        }

        // ============================================================
        // POSE UPDATES
        // ============================================================

        public void UpdateFollowerPose(float x, float y, float theta)
        {
            followerX = x;
            followerY = y;
            followerTheta = theta;
            hasFollowerPose = true;
        }

        public void UpdateLeaderPose(float x, float y, float theta)
        {
            leaderX = x;
            leaderY = y;
            leaderTheta = theta;
            hasLeaderPose = true;
            lastLeaderUpdate = DateTime.Now;

            // Update anchor point (behind leader)
            UpdateAnchorPoint();
        }

        private void UpdateAnchorPoint()
        {
            // Calculate initial anchor candidate
            float candidateX = leaderX - FollowDistance * (float)Math.Cos(leaderTheta);
            float candidateY = leaderY - FollowDistance * (float)Math.Sin(leaderTheta);

            const float ANCHOR_SAFETY_MARGIN = 12.0f;     // Min distance from obstacles
            const float ANCHOR_REPULSION_RANGE = 18.0f;   // When to push anchor

            // Check if safe
            bool isSafe = IsAnchorPositionSafe(candidateX, candidateY, ANCHOR_SAFETY_MARGIN);

            float rawAnchorX, rawAnchorY;

            if (isSafe)
            {
                rawAnchorX = candidateX;
                rawAnchorY = candidateY;
            }
            else
            {
                // Try repulsive correction
                PointF correctedAnchor = ApplyObstacleRepulsionToAnchor(
                    candidateX, candidateY,
                    ANCHOR_REPULSION_RANGE,
                    15.0f
                );

                bool correctedIsSafe = IsAnchorPositionSafe(
                    correctedAnchor.X, correctedAnchor.Y,
                    ANCHOR_SAFETY_MARGIN
                );

                if (correctedIsSafe)
                {
                    rawAnchorX = correctedAnchor.X;
                    rawAnchorY = correctedAnchor.Y;
                }
                else
                {
                    // Find alternative
                    PointF safeAnchor = FindSafeAnchorPosition(
                        candidateX, candidateY,
                        ANCHOR_SAFETY_MARGIN
                    );
                    rawAnchorX = safeAnchor.X;
                    rawAnchorY = safeAnchor.Y;
                }
            }

            // Constrain max distance
            float dx = rawAnchorX - leaderX;
            float dy = rawAnchorY - leaderY;
            float dist = (float)Math.Sqrt(dx * dx + dy * dy);

            if (dist > FollowDistance * 1.8f)
            {
                float scale = FollowDistance * 1.8f / dist;
                rawAnchorX = leaderX + dx * scale;
                rawAnchorY = leaderY + dy * scale;
            }

            // Smooth
            if (!anchorInitialized)
            {
                anchorX = rawAnchorX;
                anchorY = rawAnchorY;
                anchorInitialized = true;
            }
            else
            {
                anchorX = AnchorLowPassAlpha * rawAnchorX + (1 - AnchorLowPassAlpha) * anchorX;
                anchorY = AnchorLowPassAlpha * rawAnchorY + (1 - AnchorLowPassAlpha) * anchorY;
            }
        }

        private bool IsAnchorPositionSafe(float anchorX, float anchorY, float safetyMargin)
        {
            lock (obstacleLock)
            {
                if (obstacles.Count == 0) return true;

                foreach (var obs in obstacles)
                {
                    float dx = anchorX - obs.X;
                    float dy = anchorY - obs.Y;
                    float dist = (float)Math.Sqrt(dx * dx + dy * dy);

                    if (dist < safetyMargin)
                        return false;
                }
            }
            return true;
        }

        private PointF ApplyObstacleRepulsionToAnchor(float anchorX, float anchorY,
            float repulsionRange, float repulsionGain)
        {
            float totalRepX = 0, totalRepY = 0;

            lock (obstacleLock)
            {
                foreach (var obs in obstacles)
                {
                    float dx = anchorX - obs.X;
                    float dy = anchorY - obs.Y;
                    float dist = (float)Math.Sqrt(dx * dx + dy * dy);

                    if (dist < repulsionRange && dist > 0.1f)
                    {
                        float mag = repulsionGain * (1.0f / dist - 1.0f / repulsionRange);
                        totalRepX += mag * (dx / dist);
                        totalRepY += mag * (dy / dist);
                    }
                }
            }

            return new PointF(anchorX + totalRepX, anchorY + totalRepY);
        }

        private PointF FindSafeAnchorPosition(float unsafeX, float unsafeY, float safetyMargin)
        {
            // Try different angles around leader
            float[] angles = new float[]
            {
        leaderTheta + (float)Math.PI,       // Behind
        leaderTheta + (float)Math.PI * 0.75f,  // Left-behind
        leaderTheta + (float)Math.PI * 1.25f,  // Right-behind
        leaderTheta + (float)Math.PI * 0.5f,   // Left
        leaderTheta + (float)Math.PI * 1.5f,   // Right
            };

            foreach (float angle in angles)
            {
                float candX = leaderX + FollowDistance * (float)Math.Cos(angle);
                float candY = leaderY + FollowDistance * (float)Math.Sin(angle);

                if (IsAnchorPositionSafe(candX, candY, safetyMargin))
                    return new PointF(candX, candY);
            }

            // Try closer to leader
            for (float scale = 0.7f; scale > 0.3f; scale -= 0.1f)
            {
                float dist = FollowDistance * scale;
                float candX = leaderX - dist * (float)Math.Cos(leaderTheta);
                float candY = leaderY - dist * (float)Math.Sin(leaderTheta);

                if (IsAnchorPositionSafe(candX, candY, safetyMargin))
                    return new PointF(candX, candY);
            }

            // Last resort - leader position
            return new PointF(leaderX, leaderY);
        }


        // ============================================================
        // OBSTACLE UPDATE
        // ============================================================

        public void UpdateObstacles(List<PointF> newObstacles)
        {
            lock (obstacleLock)
            {
                obstacles.Clear();
                obstacles.AddRange(newObstacles);
            }
        }

        // ============================================================
        // POTENTIAL FIELD CALCULATIONS
        // ============================================================

        private PointF CalculateAttractiveForce()
        {
            if (!anchorInitialized) return PointF.Empty;

            float dx = anchorX - followerX;
            float dy = anchorY - followerY;
            float dist = (float)Math.Sqrt(dx * dx + dy * dy);

            if (dist < 0.01f) return PointF.Empty;

            float nx = dx / dist;
            float ny = dy / dist;

            float forceMag = AttractiveGain * dist;
            forceMag = Math.Min(forceMag, AttractiveMaxForce);

            return new PointF(forceMag * nx, forceMag * ny);
        }

        private PointF CalculateRepulsiveForce()
        {
            float totalFx = 0, totalFy = 0;

            List<PointF> obstaclesCopy;
            lock (obstacleLock)
            {
                obstaclesCopy = new List<PointF>(obstacles);
            }

            foreach (var obs in obstaclesCopy)
            {
                float dx = followerX - obs.X;
                float dy = followerY - obs.Y;
                float dist = (float)Math.Sqrt(dx * dx + dy * dy);

                if (dist >= ObstacleInfluenceRange || dist < 0.1f)
                    continue;

                float nx = dx / dist;
                float ny = dy / dist;

                float invD = 1.0f / dist;
                float invD0 = 1.0f / ObstacleInfluenceRange;
                float forceMag = RepulsiveGain * (invD - invD0) * invD * invD;
                forceMag = Math.Min(forceMag, RepulsiveMaxForce);

                totalFx += forceMag * nx;
                totalFy += forceMag * ny;
            }

            return new PointF(totalFx, totalFy);
        }

        /// <summary>
        /// Repulsive force from Leader (avoid collision with leader)
        /// </summary>
        private PointF CalculateLeaderAvoidanceForce()
        {
            if (!hasLeaderPose) return PointF.Empty;

            float dx = followerX - leaderX;
            float dy = followerY - leaderY;
            float dist = (float)Math.Sqrt(dx * dx + dy * dy);

            if (dist >= LeaderAvoidanceRange || dist < 0.1f)
                return PointF.Empty;

            float nx = dx / dist;
            float ny = dy / dist;

            float invD = 1.0f / dist;
            float invD0 = 1.0f / LeaderAvoidanceRange;
            float forceMag = LeaderAvoidanceGain * (invD - invD0) * invD * invD;

            return new PointF(forceMag * nx, forceMag * ny);
        }

        private void CalculateControlOutput(out float linearVel, out float angularVel)
        {
            linearVel = 0;
            angularVel = 0;

            if (!hasFollowerPose || !hasLeaderPose || !anchorInitialized)
                return;

            // Check leader timeout
            if (DateTime.Now - lastLeaderUpdate > leaderTimeout)
            {
                OnDebugMessage?.Invoke("Leader timeout - stopping");
                currentState = FollowerState.Idle;
                return;
            }

            // Calculate distance to leader
            float dxL = leaderX - followerX;
            float dyL = leaderY - followerY;
            float distToLeader = (float)Math.Sqrt(dxL * dxL + dyL * dyL);

            // Calculate distance to anchor
            float dxA = anchorX - followerX;
            float dyA = anchorY - followerY;
            float distToAnchor = (float)Math.Sqrt(dxA * dxA + dyA * dyA);

            // ========== STATE MACHINE (IMPROVED) ==========
            // CRITICAL FIX: Tambahkan hysteresis untuk menghindari oscillation

            const float STOP_THRESHOLD = 25.0f;      // Stop jika < 8cm dari leader (lebih kecil dari 10cm default)
            const float RESUME_THRESHOLD = 30.0f;   // Resume jika > 18cm (lebih kecil dari 20cm default)
            const float ANCHOR_GOAL_TOLERANCE = 15.0f; // Berhasil jika dalam 8cm dari anchor
            float stopBuffer = 10.0f;
            switch (currentState)
            {
                case FollowerState.Idle:
                    if (distToLeader > RESUME_THRESHOLD)
                    {
                        currentState = FollowerState.Catching;
                        OnStatusChanged?.Invoke("Catching up to leader");
                    }
                    break;

                case FollowerState.Following:
                    if (distToLeader <= (StopDistance + stopBuffer))  // 10 + 5 = 15cm
                    {
                        currentState = FollowerState.Stopped;
                        OnStatusChanged?.Invoke($"Too close ({distToLeader:F1}cm) - stopping");
                        linearVel = 0;
                        angularVel = 0;
                        return;
                    }
                    break;


                case FollowerState.Stopped:
                    // HYSTERESIS: Butuh jarak lebih jauh untuk resume (cegah oscillation)
                    if (distToLeader > RESUME_THRESHOLD)
                    {
                        currentState = FollowerState.Following;
                        OnStatusChanged?.Invoke("Resuming follow");
                    }
                    break;

                case FollowerState.Catching:
                    if (distToAnchor < ANCHOR_GOAL_TOLERANCE)
                    {
                        currentState = FollowerState.Following;
                        OnStatusChanged?.Invoke("Caught up - following");
                    }
                    break;
            }

            // Don't move if stopped or idle
            if (currentState == FollowerState.Stopped || currentState == FollowerState.Idle)
            {
                return;
            }

            // ========== FORCE CALCULATION ==========
            PointF attractiveForce = CalculateAttractiveForce();
            PointF repulsiveForce = CalculateRepulsiveForce();
            PointF leaderAvoidForce = CalculateLeaderAvoidanceForce();

            float totalFx = attractiveForce.X + repulsiveForce.X + leaderAvoidForce.X;
            float totalFy = attractiveForce.Y + repulsiveForce.Y + leaderAvoidForce.Y;
            float forceMag = (float)Math.Sqrt(totalFx * totalFx + totalFy * totalFy);

            if (forceMag < 0.01f)
                return;

            // ========== CONVERT TO VELOCITY ==========
            float desiredHeading = (float)Math.Atan2(totalFy, totalFx);
            float headingError = WrapPi(desiredHeading - followerTheta);

            // ========== EMERGENCY OBSTACLE STOP (ONLY) ==========
            float minObstDist = GetMinObstacleDistance();
            if (minObstDist < EmergencyStopDistance)
            {
                linearVel = 0;
                angularVel = 0;
                return;
            }

            // ========== TURN-IN-PLACE LOGIC (RELAXED) ==========
            // CRITICAL FIX: Hanya turn-in-place jika masih JAUH dari anchor
            // Jangan turn-in-place kalau sudah dekat anchor (biarkan maju sambil belok)
            bool allowTurnInPlace = (distToAnchor > 35.0f && distToLeader > RESUME_THRESHOLD);

            if (Math.Abs(headingError) > TurnInPlaceThreshold && allowTurnInPlace)
            {
                linearVel = 0;
                angularVel = Math.Sign(headingError) * MaxAngularVelocity * 0.3f; // Lebih smooth
            }
            else
            {
                // ========== NORMAL FOLLOWING ==========
                // Angle reduction factor (jangan terlalu harsh)
                float angleReduction = 1.0f - Math.Abs(headingError) / (float)Math.PI;
                angleReduction = Math.Max(0.5f, angleReduction); // Minimal 50% speed

                // Base linear velocity dari force magnitude
                linearVel = (forceMag / AttractiveMaxForce) * MaxLinearVelocity * angleReduction;
                linearVel = Math.Min(linearVel, MaxLinearVelocity);

                // ========== SMOOTH SLOWDOWN NEAR ANCHOR ==========
                // CRITICAL FIX: Slowdown gradual, bukan hard stop
                float slowdownRadius = 15.0f; // Mulai pelan dalam 15cm dari anchor
                if (distToAnchor < slowdownRadius)
                {
                    float slowdownFactor = distToAnchor / slowdownRadius;
                    slowdownFactor = Math.Max(0.2f, slowdownFactor); // Minimal 30% speed
                    linearVel *= slowdownFactor;
                }

                // Minimum velocity (hanya jika masih cukup jauh)
                if (distToAnchor > 10.0f && linearVel > 0.1f && linearVel < MinLinearVelocity)
                {
                    linearVel = MinLinearVelocity;
                }

                // Slow down near obstacles (smooth)
                if (minObstDist < ObstacleInfluenceRange)
                {
                    float obstacleFactor = minObstDist / ObstacleInfluenceRange;
                    linearVel *= Math.Max(0.4f, obstacleFactor);
                }

                // Angular velocity (smooth control)
                float Kw = 1.5f; // Turunkan dari 1.8 supaya tidak terlalu aggressive
                angularVel = Kw * headingError;
                angularVel = Clamp(angularVel, -MaxAngularVelocity, MaxAngularVelocity);

                // Deadband untuk mengurangi jitter
                if (Math.Abs(headingError) < AngleDeadband)
                    angularVel = 0;
            }

            // ========== SMOOTHING ==========
            filteredLinearVel = VelocityLowPassAlpha * linearVel + (1 - VelocityLowPassAlpha) * filteredLinearVel;
            filteredAngularVel = VelocityLowPassAlpha * angularVel + (1 - VelocityLowPassAlpha) * filteredAngularVel;

            linearVel = filteredLinearVel;
            angularVel = filteredAngularVel;

            OnVelocityCommand?.Invoke(linearVel, angularVel);
        }

        private float GetMinObstacleDistance()
        {
            float minDist = float.MaxValue;

            lock (obstacleLock)
            {
                foreach (var obs in obstacles)
                {
                    float dx = followerX - obs.X;
                    float dy = followerY - obs.Y;
                    float dist = (float)Math.Sqrt(dx * dx + dy * dy);
                    if (dist < minDist) minDist = dist;
                }
            }

            return minDist;
        }

        // ============================================================
        // CONTROL LOOP
        // ============================================================

        public void StartControl()
        {
            if (isRunning) return;

            isRunning = true;
            currentState = FollowerState.Idle;
            controlCts = new CancellationTokenSource();

            controlTask = Task.Run(async () =>
            {
                OnStatusChanged?.Invoke("Follower Reactive PF started");

                // Connect and send START (once after connected)
                bool startSent = false;
                EnsureConnection();
                if (followerWriter == null)
                    OnStatusChanged?.Invoke($"Follower connect failed to {followerIP}:{followerPort}");

                while (!controlCts.Token.IsCancellationRequested)
                {
                    try
                    {
                        // (Re)connect if needed
                        if (followerWriter == null || followerClient == null || !followerClient.Connected)
                        {
                            EnsureConnection();
                            if (followerWriter == null)
                            {
                                if (!startSent)
                                    OnStatusChanged?.Invoke($"Follower connect failed to {followerIP}:{followerPort}");
                                await Task.Delay(300, controlCts.Token);
                                continue;
                            }
                        }

                        if (!startSent)
                        {
                            SendCommand("START");
                            SendCommand("HELP"); // debug: should appear on Serial Monitor if connected
                            startSent = true;
                        }

                        if (hasFollowerPose && hasLeaderPose)
                        {
                            CalculateControlOutput(out float v, out float w);
                            SendVelocityCommand(v, w);

                            // Also send SETLEADER for odometry sync
                            // SendSetLeader(); // disabled: jangan spam SETLEADER saat PF follower aktif
                        }

                        await Task.Delay(controlLoopIntervalMs, controlCts.Token);
                    }
                    catch (OperationCanceledException)
                    {
                        break;
                    }
                    catch (Exception ex)
                    {
                        OnDebugMessage?.Invoke($"Follower control error: {ex.Message}");
                    }
                }

                SendStopCommand();
                OnStatusChanged?.Invoke("Follower Reactive PF stopped");
            }, controlCts.Token);
        }

        public void StopControl()
        {
            if (!isRunning) return;

            isRunning = false;
            controlCts?.Cancel();

            try { controlTask?.Wait(1000); } catch { }

            SendStopCommand();
            SendCommand("STOP");
        }

        // ============================================================
        // NETWORK COMMUNICATION
        // ============================================================

        private void EnsureConnection()
        {
            if (followerClient != null && followerClient.Connected && followerWriter != null)
                return;

            try
            {
                followerClient?.Close();
                followerClient = new TcpClient { NoDelay = true };

                var ar = followerClient.BeginConnect(followerIP, followerPort, null, null);
                if (!ar.AsyncWaitHandle.WaitOne(1000))
                    throw new Exception("Connection timeout");

                followerClient.EndConnect(ar);
                followerWriter = new StreamWriter(followerClient.GetStream(), new UTF8Encoding(false)) { AutoFlush = true };

                OnDebugMessage?.Invoke($"Connected to Follower at {followerIP}:{followerPort}");
            }
            catch (Exception ex)
            {
                followerWriter = null;
                followerClient = null;
                OnDebugMessage?.Invoke($"Follower connection failed: {ex.Message}");
            }
        }

        private void SendCommand(string cmd)
        {
            try
            {
                EnsureConnection();
                followerWriter?.WriteLine(cmd);
            }
            catch { }
        }

        private void SendVelocityCommand(float v, float w)
        {
            try
            {
                // deadband kecil supaya tidak spin karena noise
                if (Math.Abs(v) < 0.05f) v = 0f;      // cm/s
                if (Math.Abs(w) < 0.03f) w = 0f;      // rad/s

                EnsureConnection();
                if (followerWriter != null)
                {
                    // Follower firmware menggunakan SETVEL untuk direct velocity control
                    string cmd = $"SETVEL {v.ToString("F2", CI)} {w.ToString("F3", CI)}";
                    followerWriter.WriteLine(cmd);
                }
            }
            catch (Exception ex)
            {
                OnDebugMessage?.Invoke($"SendVelocity error: {ex.Message}");
                followerWriter = null;
                followerClient = null;
            }
        }

        private void SendSetLeader()
        {
            try
            {
                EnsureConnection();
                if (followerWriter != null && hasLeaderPose)
                {
                    string cmd = $"SETLEADER {leaderX.ToString("F2", CI)} {leaderY.ToString("F2", CI)} {leaderTheta.ToString("F4", CI)}";
                    followerWriter.WriteLine(cmd);
                }
            }
            catch { }
        }

        private void SendStopCommand()
        {
            try
            {
                EnsureConnection();
                followerWriter?.WriteLine("SETVEL 0.00 0.000");
            }
            catch { }
        }

        // ============================================================
        // HELPERS
        // ============================================================

        private float WrapPi(float a)
        {
            while (a > Math.PI) a -= 2 * (float)Math.PI;
            while (a < -Math.PI) a += 2 * (float)Math.PI;
            return a;
        }

        private float Clamp(float v, float min, float max) => Math.Max(min, Math.Min(max, v));

        // Getters
        public PointF GetFollowerPosition() => new PointF(followerX, followerY);
        public PointF GetAnchorPosition() => new PointF(anchorX, anchorY);
        public string GetCurrentState() => currentState.ToString();
        public bool IsRunning => isRunning;

        public void Dispose()
        {
            StopControl();
            try { followerWriter?.Close(); } catch { }
            try { followerClient?.Close(); } catch { }
            controlCts?.Dispose();
        }
    }
}