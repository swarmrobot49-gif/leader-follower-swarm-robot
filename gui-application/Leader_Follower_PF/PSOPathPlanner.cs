using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace HybridDijkstraPotentialField
{
    /// <summary>
    /// Particle Swarm Optimization Path Planner
    /// Bio-inspired algorithm untuk path planning dengan multi-objective optimization
    /// </summary>
    public class PSOPathPlanner
    {
        // === PSO Parameters (need tuning!) ===
        private const int SWARM_SIZE = 30;              // Jumlah particles
        private const int MAX_ITERATIONS = 100;         // Max iterations
        private const float INERTIA_WEIGHT = 0.729f;    // w: Inertia (momentum)
        private const float COGNITIVE_WEIGHT = 1.49f;   // c1: Personal best influence
        private const float SOCIAL_WEIGHT = 1.49f;      // c2: Global best influence
        private const int PATH_LENGTH = 15;             // Jumlah waypoints per path

        // Fitness weights (untuk multi-objective)
        private const float WEIGHT_PATH_LENGTH = 0.4f;
        private const float WEIGHT_COLLISION = 0.4f;
        private const float WEIGHT_SMOOTHNESS = 0.2f;

        private Point startGrid;
        private Point goalGrid;


        private CellType[,] gridData;
        private int gridCols, gridRows;
        private float gridCellSizeCm;
        private Random random = new Random();

        public PSOPathPlanner(CellType[,] grid, int cols, int rows, float cellSize)
        {
            this.gridData = grid;
            this.gridCols = cols;
            this.gridRows = rows;
            this.gridCellSizeCm = cellSize;
        }

        /// <summary>
        /// Particle representation
        /// Setiap particle = satu kandidat path (sequence of waypoints)
        /// </summary>
        private class Particle
        {
            public List<PointF> Position;      // Current path (in cm)
            public List<PointF> Velocity;      // Velocity for each waypoint
            public List<PointF> PersonalBest;  // Best position this particle found
            public float PersonalBestFitness;  // Fitness of personal best
            public float CurrentFitness;

            public Particle(int pathLength)
            {
                Position = new List<PointF>(pathLength);
                Velocity = new List<PointF>(pathLength);
                PersonalBest = new List<PointF>(pathLength);
                PersonalBestFitness = float.MaxValue;
                CurrentFitness = float.MaxValue;
            }
        }

        /// <summary>
        /// Main PSO path finding function
        /// </summary>
        public List<Point> FindPath(Point startGrid, Point goalGrid)
        {
            this.startGrid = startGrid;
            this.goalGrid = goalGrid;
            PointF startCm = GridToCm(startGrid);
            PointF goalCm = GridToCm(goalGrid);

            // 1. Initialize swarm
            List<Particle> swarm = InitializeSwarm(startCm, goalCm);

            // 2. Find initial global best
            Particle globalBest = swarm[0];
            foreach (var particle in swarm)
            {
                if (particle.CurrentFitness < globalBest.CurrentFitness)
                {
                    globalBest = particle;
                }
            }

            // 3. PSO Main Loop
            for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++)
            {
                foreach (var particle in swarm)
                {
                    // Update velocity
                    UpdateVelocity(particle, globalBest);

                    // Update position
                    UpdatePosition(particle, startCm, goalCm);

                    // Evaluate fitness
                    particle.CurrentFitness = EvaluateFitness(particle.Position, startCm, goalCm);

                    // Update personal best
                    if (particle.CurrentFitness < particle.PersonalBestFitness)
                    {
                        particle.PersonalBestFitness = particle.CurrentFitness;
                        particle.PersonalBest = new List<PointF>(particle.Position);
                    }

                    // Update global best
                    if (particle.CurrentFitness < globalBest.CurrentFitness)
                    {
                        globalBest = particle;
                    }
                }

                // Optional: Adaptive inertia weight (decrease over time)
                // INERTIA_WEIGHT = 0.9f - (iteration / (float)MAX_ITERATIONS) * 0.5f;

                // Early termination if converged
                if (globalBest.CurrentFitness < 0.01f)
                {
                    break; // Found excellent solution
                }
            }

            // 4. Convert best path to grid coordinates
            List<Point> finalPath = ConvertToGridPath(globalBest.PersonalBest);

            return finalPath;
        }

        /// <summary>
        /// Initialize particle swarm with random paths
        /// </summary>
        private List<Particle> InitializeSwarm(PointF start, PointF goal)
        {
            List<Particle> swarm = new List<Particle>();

            for (int i = 0; i < SWARM_SIZE; i++)
            {
                Particle particle = new Particle(PATH_LENGTH);

                // Generate random path from start to goal
                for (int j = 0; j < PATH_LENGTH; j++)
                {
                    float t = j / (float)(PATH_LENGTH - 1); // Interpolation parameter [0,1]
                    
                    // Linear interpolation with random noise
                    float x = Lerp(start.X, goal.X, t) + RandomGaussian(0, gridCellSizeCm);
                    float y = Lerp(start.Y, goal.Y, t) + RandomGaussian(0, gridCellSizeCm);

                    // Clamp to arena bounds
                    x = Clamp(x, 0, gridCols * gridCellSizeCm);
                    y = Clamp(y, 0, gridRows * gridCellSizeCm);

                    particle.Position.Add(new PointF(x, y));

                    // Random initial velocity
                    particle.Velocity.Add(new PointF(
                        RandomGaussian(0, gridCellSizeCm * 0.1f),
                        RandomGaussian(0, gridCellSizeCm * 0.1f)
                    ));
                }

                // Evaluate initial fitness
                particle.CurrentFitness = EvaluateFitness(particle.Position, start, goal);
                particle.PersonalBestFitness = particle.CurrentFitness;
                particle.PersonalBest = new List<PointF>(particle.Position);

                swarm.Add(particle);
            }

            return swarm;
        }

        /// <summary>
        /// Update particle velocity (PSO core equation)
        /// v(t+1) = w*v(t) + c1*r1*(pbest - x(t)) + c2*r2*(gbest - x(t))
        /// </summary>
        private void UpdateVelocity(Particle particle, Particle globalBest)
        {
            for (int i = 0; i < PATH_LENGTH; i++)
            {
                float r1 = (float)random.NextDouble();
                float r2 = (float)random.NextDouble();

                // Inertia component
                float vx = INERTIA_WEIGHT * particle.Velocity[i].X;
                float vy = INERTIA_WEIGHT * particle.Velocity[i].Y;

                // Cognitive component (personal best)
                vx += COGNITIVE_WEIGHT * r1 * (particle.PersonalBest[i].X - particle.Position[i].X);
                vy += COGNITIVE_WEIGHT * r1 * (particle.PersonalBest[i].Y - particle.Position[i].Y);

                // Social component (global best)
                vx += SOCIAL_WEIGHT * r2 * (globalBest.PersonalBest[i].X - particle.Position[i].X);
                vy += SOCIAL_WEIGHT * r2 * (globalBest.PersonalBest[i].Y - particle.Position[i].Y);

                // Velocity clamping (prevent explosion)
                float maxVelocity = gridCellSizeCm * 2;
                vx = Clamp(vx, -maxVelocity, maxVelocity);
                vy = Clamp(vy, -maxVelocity, maxVelocity);

                particle.Velocity[i] = new PointF(vx, vy);
            }
        }

        /// <summary>
        /// Update particle position
        /// x(t+1) = x(t) + v(t+1)
        /// </summary>
        private void UpdatePosition(Particle particle, PointF start, PointF goal)
        {
            for (int i = 0; i < PATH_LENGTH; i++)
            {
                // Don't move start and goal waypoints
                if (i == 0 || i == PATH_LENGTH - 1)
                {
                    particle.Position[0] = start;
                    particle.Position[PATH_LENGTH - 1] = goal;
                    continue;
                }

                // Update intermediate waypoints
                float newX = particle.Position[i].X + particle.Velocity[i].X;
                float newY = particle.Position[i].Y + particle.Velocity[i].Y;

                // Clamp to arena bounds
                newX = Clamp(newX, 0, gridCols * gridCellSizeCm);
                newY = Clamp(newY, 0, gridRows * gridCellSizeCm);

                particle.Position[i] = new PointF(newX, newY);
            }
        }

        /// <summary>
        /// Evaluate fitness of a path (lower = better)
        /// Multi-objective: path length + collision penalty + smoothness
        /// </summary>
        private float EvaluateFitness(List<PointF> path, PointF start, PointF goal)
        {
            // 1. Path length penalty
            float pathLength = CalculatePathLength(path);
            float lengthPenalty = pathLength / (Distance(start, goal) + 1.0f); // Normalized

            // 2. Collision penalty
            float collisionPenalty = CalculateCollisionPenalty(path);

            // 3. Smoothness penalty (penalize sharp turns)
            float smoothnessPenalty = CalculateSmoothnessPenalty(path);

            // Weighted sum
            float fitness = WEIGHT_PATH_LENGTH * lengthPenalty +
                           WEIGHT_COLLISION * collisionPenalty +
                           WEIGHT_SMOOTHNESS * smoothnessPenalty;

            return fitness;
        }

        /// <summary>
        /// Calculate total path length
        /// </summary>
        private float CalculatePathLength(List<PointF> path)
        {
            float totalLength = 0;
            for (int i = 1; i < path.Count; i++)
            {
                totalLength += Distance(path[i - 1], path[i]);
            }
            return totalLength;
        }

        /// <summary>
        /// Calculate collision penalty (how many segments intersect obstacles)
        /// </summary>
        private float CalculateCollisionPenalty(List<PointF> path)
        {
            float penalty = 0;

            for (int i = 1; i < path.Count; i++)
            {
                // Check line segment from path[i-1] to path[i]
                int collisions = CountCollisionsOnSegment(path[i - 1], path[i]);
                penalty += collisions * 10.0f; // Heavy penalty for collisions
            }

            return penalty;
        }

        /// <summary>
        /// Count how many grid cells along a segment are obstacles
        /// </summary>
        private int CountCollisionsOnSegment(PointF p1, PointF p2)
        {
            int collisions = 0;
            
            // Bresenham-like sampling along line
            int samples = (int)(Distance(p1, p2) / (gridCellSizeCm * 0.5f));
            samples = Math.Max(samples, 2);

            for (int i = 0; i <= samples; i++)
            {
                float t = i / (float)samples;
                PointF sample = new PointF(
                    Lerp(p1.X, p2.X, t),
                    Lerp(p1.Y, p2.Y, t)
                );

                Point gridPos = CmToGrid(sample);
                if (IsObstacle(gridPos))
                {
                    collisions++;
                }
            }

            return collisions;
        }

        /// <summary>
        /// Calculate smoothness penalty (penalize sharp turns)
        /// </summary>
        private float CalculateSmoothnessPenalty(List<PointF> path)
        {
            float penalty = 0;

            for (int i = 1; i < path.Count - 1; i++)
            {
                // Calculate angle between three consecutive points
                PointF v1 = new PointF(path[i].X - path[i - 1].X, path[i].Y - path[i - 1].Y);
                PointF v2 = new PointF(path[i + 1].X - path[i].X, path[i + 1].Y - path[i].Y);

                float angle = AngleBetween(v1, v2);
                
                // Penalize sharp turns (> 90 degrees)
                if (Math.Abs(angle) > Math.PI / 2)
                {
                    penalty += (float)Math.Abs(angle);
                }
            }

            return penalty / path.Count;
        }

        /// <summary>
        /// Convert continuous path to grid waypoints
        /// </summary>
        private List<Point> ConvertToGridPath(List<PointF> continuousPath)
        {
            List<Point> gridPath = new List<Point>();

            for (int i = 0; i < continuousPath.Count; i++)
            {
                Point gridPos = CmToGrid(continuousPath[i]);

                // Pastikan waypoint pertama & terakhir persis di start/goal
                if (i == 0)
                    gridPos = startGrid;
                else if (i == continuousPath.Count - 1)
                    gridPos = goalGrid;

                // Skip kalau keluar grid atau di dalam obstacle
                if (!IsValidGridPosition(gridPos))
                    continue;

                if (IsObstacle(gridPos))
                    continue;

                // Tambahkan hanya kalau beda dari waypoint sebelumnya
                if (gridPath.Count == 0 || gridPath[gridPath.Count - 1] != gridPos)
                {
                    gridPath.Add(gridPos);
                }
            }

            // Jamin path punya start & goal kalau memungkinkan
            if (gridPath.Count == 0)
                return gridPath;

            if (gridPath[0] != startGrid)
                gridPath.Insert(0, startGrid);

            if (gridPath[gridPath.Count - 1] != goalGrid)
                gridPath.Add(goalGrid);

            return gridPath;
        }

        // === Helper Functions ===

        private PointF GridToCm(Point grid)
        {
            return new PointF(
                grid.X * gridCellSizeCm + gridCellSizeCm / 2.0f,
                grid.Y * gridCellSizeCm + gridCellSizeCm / 2.0f
            );
        }

        private Point CmToGrid(PointF cm)
        {
            return new Point(
                (int)(cm.X / gridCellSizeCm),
                (int)(cm.Y / gridCellSizeCm)
            );
        }

        private bool IsValidGridPosition(Point grid)
        {
            return grid.X >= 0 && grid.X < gridCols && 
                   grid.Y >= 0 && grid.Y < gridRows;
        }

        private bool IsObstacle(Point grid)
        {
            if (!IsValidGridPosition(grid))
                return true; // Out of bounds = obstacle

            return gridData[grid.X, grid.Y] == CellType.Obstacle;
        }

        private float Distance(PointF a, PointF b)
        {
            float dx = b.X - a.X;
            float dy = b.Y - a.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        private float Lerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        private float Clamp(float value, float min, float max)
        {
            return value < min ? min : (value > max ? max : value);
        }

        private float RandomGaussian(float mean, float stdDev)
        {
            // Box-Muller transform
            double u1 = random.NextDouble();
            double u2 = random.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
            return mean + stdDev * (float)randStdNormal;
        }

        private float AngleBetween(PointF v1, PointF v2)
        {
            float dot = v1.X * v2.X + v1.Y * v2.Y;
            float mag1 = (float)Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y);
            float mag2 = (float)Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y);

            if (mag1 < 0.001f || mag2 < 0.001f)
                return 0;

            float cosAngle = dot / (mag1 * mag2);
            cosAngle = Clamp(cosAngle, -1.0f, 1.0f);
            return (float)Math.Acos(cosAngle);
        }
    }
}
