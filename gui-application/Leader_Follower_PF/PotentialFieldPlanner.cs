using System;
using System.Collections.Generic;
using System.Drawing;

namespace HybridDijkstraPotentialField
{
    public class PotentialFieldPlanner
    {
        private CellType[,] grid;
        private int cols;
        private int rows;
        private float cellSizeCm; 

        // === TUNING PARAMETERS - BALANCED: AVOID OBSTACLES + REACH GOAL ===
        private const float ATTRACTIVE_GAIN = 65.5f;          // Tarikan ke goal
        private const float REPULSIVE_GAIN = 100.0f;         // Tolakan dari obstacle (DINAIKKAN kembali!)
        private const float OBSTACLE_INFLUENCE_RANGE = 7.0f; // Range pengaruh obstacle (DIPERLEBAR)
        private const float STEP_SIZE = 0.1f;                // Ukuran langkah (DIKECILKAN sedikit)
        private const int MAX_ITERATIONS = 2500;             // Max iterasi (DINAIKKAN lagi)
        private const float GOAL_THRESHOLD = 0.6f;           // Threshold sampai goal
        private const float MIN_FORCE = 0.002f;              // Minimum force

        public PotentialFieldPlanner(CellType[,] gridData, int gridCols, int gridRows, float cellSize)
        {
            this.grid = gridData;
            this.cols = gridCols;
            this.rows = gridRows;
            this.cellSizeCm = cellSize;
        }

        public List<Point> FindPath(Point start, Point goal)
        {
            List<Point> path = new List<Point>();

            // Validasi
            if (!IsValid(start) || !IsValid(goal))
            {
                System.Diagnostics.Debug.WriteLine("[PF] Invalid start or goal");
                return null;
            }

            if (grid[start.X, start.Y] == CellType.Obstacle ||
                grid[goal.X, goal.Y] == CellType.Obstacle)
            {
                System.Diagnostics.Debug.WriteLine("[PF] Start/goal in obstacle");
                return null;
            }

            // Kumpulkan semua posisi obstacle
            List<PointF> obstacles = new List<PointF>();
            for (int c = 0; c < cols; c++)
            {
                for (int r = 0; r < rows; r++)
                {
                    if (grid[c, r] == CellType.Obstacle)
                    {
                        obstacles.Add(new PointF(c, r));
                    }
                }
            }

            System.Diagnostics.Debug.WriteLine($"[PF] Starting pathfinding. Found {obstacles.Count} obstacles");

            // Posisi saat ini (float untuk smooth movement)
            PointF current = new PointF(start.X + 0.5f, start.Y + 0.5f);
            path.Add(start);

            HashSet<Point> visited = new HashSet<Point>();
            visited.Add(start);

            int stuckCounter = 0;

            for (int iter = 0; iter < MAX_ITERATIONS; iter++)
            {
                // Cek jarak ke goal
                float distToGoal = CalcDistance(current, new PointF(goal.X + 0.5f, goal.Y + 0.5f));

                if (distToGoal < GOAL_THRESHOLD)
                {
                    path.Add(goal);
                    System.Diagnostics.Debug.WriteLine($"[PF] SUCCESS! Path found with {path.Count} waypoints in {iter} iterations");
                    return path;
                }

                // === HITUNG ATTRACTIVE FORCE (ke goal) ===
                PointF attractiveForce = CalcAttractive(current, goal);

                // === HITUNG REPULSIVE FORCE (dari obstacles) ===
                // Kurangi repulsive multiplier HANYA SEDIKIT saat sangat dekat dengan goal
                float repulsiveMultiplier = 1.0f;
                if (distToGoal < 2.0f) // Hanya saat SANGAT SANGAT dekat (dalam 2 cells)
                {
                    repulsiveMultiplier = 0.6f; // Masih 60% repulsive (tidak terlalu rendah!)
                }
                else if (distToGoal < 4.0f) // Saat cukup dekat
                {
                    repulsiveMultiplier = 0.8f; // 80% repulsive
                }

                PointF repulsiveForce = CalcRepulsive(current, obstacles, repulsiveMultiplier);

                // === TOTAL FORCE ===
                PointF totalForce = new PointF(
                    attractiveForce.X + repulsiveForce.X,
                    attractiveForce.Y + repulsiveForce.Y
                );

                // Normalize force
                float magnitude = (float)Math.Sqrt(totalForce.X * totalForce.X + totalForce.Y * totalForce.Y);

                if (magnitude < MIN_FORCE)
                {
                    // Stuck in local minimum - random perturbation
                    Random rand = new Random(iter + (int)current.X * 1000 + (int)current.Y);
                    totalForce.X += (float)(rand.NextDouble() - 0.5) * 3.0f;
                    totalForce.Y += (float)(rand.NextDouble() - 0.5) * 3.0f;
                    magnitude = (float)Math.Sqrt(totalForce.X * totalForce.X + totalForce.Y * totalForce.Y);
                    System.Diagnostics.Debug.WriteLine($"[PF] Stuck at iter {iter}, applying perturbation");
                }

                if (magnitude > 0)
                {
                    totalForce.X = (totalForce.X / magnitude) * STEP_SIZE;
                    totalForce.Y = (totalForce.Y / magnitude) * STEP_SIZE;
                }

                // Update posisi
                PointF nextPos = new PointF(
                    current.X + totalForce.X,
                    current.Y + totalForce.Y
                );

                // Clamp ke bounds
                nextPos.X = Math.Max(0.5f, Math.Min(cols - 0.5f, nextPos.X));
                nextPos.Y = Math.Max(0.5f, Math.Min(rows - 0.5f, nextPos.Y));

                // === HARD CONSTRAINT: JANGAN MASUK OBSTACLE ===
                Point nextCell = new Point((int)Math.Floor(nextPos.X), (int)Math.Floor(nextPos.Y));

                // Cek apakah akan masuk obstacle
                if (IsValid(nextCell) && grid[nextCell.X, nextCell.Y] == CellType.Obstacle)
                {
                    // FORBIDDEN! Jangan pindah ke sini
                    System.Diagnostics.Debug.WriteLine($"[PF] BLOCKED at ({nextCell.X},{nextCell.Y}) - rejecting move");

                    // Tingkatkan repulsive SANGAT drastis & kurangi step
                    repulsiveForce = CalcRepulsive(current, obstacles, multiplier: 50.0f); // SUPER HIGH!
                    totalForce.X = attractiveForce.X * 0.3f + repulsiveForce.X; // Kurangi attractive
                    totalForce.Y = attractiveForce.Y * 0.3f + repulsiveForce.Y;

                    magnitude = (float)Math.Sqrt(totalForce.X * totalForce.X + totalForce.Y * totalForce.Y);
                    if (magnitude > 0)
                    {
                        totalForce.X = (totalForce.X / magnitude) * STEP_SIZE * 0.1f; // Step sangat kecil
                        totalForce.Y = (totalForce.Y / magnitude) * STEP_SIZE * 0.1f;
                    }

                    nextPos.X = current.X + totalForce.X;
                    nextPos.Y = current.Y + totalForce.Y;
                    nextPos.X = Math.Max(0.5f, Math.Min(cols - 0.5f, nextPos.X));
                    nextPos.Y = Math.Max(0.5f, Math.Min(rows - 0.5f, nextPos.Y));
                    nextCell = new Point((int)Math.Floor(nextPos.X), (int)Math.Floor(nextPos.Y));

                    // Jika MASIH di obstacle setelah adjustment, JANGAN GERAK SAMA SEKALI
                    if (IsValid(nextCell) && grid[nextCell.X, nextCell.Y] == CellType.Obstacle)
                    {
                        nextPos = current; // STAY di posisi sekarang
                        nextCell = new Point((int)Math.Floor(current.X), (int)Math.Floor(current.Y));
                        System.Diagnostics.Debug.WriteLine($"[PF] Cannot move - staying at current position");
                        stuckCounter++; // Increment stuck counter disini saja
                    }

                    // HAPUS stuckCounter++ yang ada di baris berikutnya (sudah di atas)
                    if (stuckCounter > 80) // Naikkan threshold dari 50 ke 80
                    {
                        System.Diagnostics.Debug.WriteLine($"[PF] Stuck too long ({stuckCounter} times) - cannot find valid path around obstacles");
                        break;
                    }
                }
                else
                {
                    stuckCounter = 0; // Reset jika tidak ada collision
                }

                // Double-check: pastikan tidak di obstacle
                if (IsValid(nextCell) && grid[nextCell.X, nextCell.Y] != CellType.Obstacle)
                {
                    // OK untuk bergerak
                    current = nextPos;
                    // Jangan reset stuck counter di sini, sudah di-handle di atas
                }
                else
                {
                    // Masih di obstacle - sudah handled di atas, jangan increment lagi
                    System.Diagnostics.Debug.WriteLine($"[PF] Final check: staying at ({current.X:F1},{current.Y:F1})");
                    // JANGAN tambah stuckCounter++ lagi di sini!
                }

                // Add to path jika cell baru DAN BUKAN OBSTACLE
                if (!visited.Contains(nextCell) && IsValid(nextCell))
                {
                    // CRITICAL CHECK: Jangan tambahkan jika cell adalah obstacle!
                    if (grid[nextCell.X, nextCell.Y] != CellType.Obstacle)
                    {
                        path.Add(nextCell);
                        visited.Add(nextCell);
                    }
                    else
                    {
                        // Cell adalah obstacle - JANGAN TAMBAHKAN KE PATH!
                        System.Diagnostics.Debug.WriteLine($"[PF] Skipping obstacle cell ({nextCell.X},{nextCell.Y}) - NOT adding to path");
                        visited.Add(nextCell); // Tapi tetap mark sebagai visited agar tidak loop
                    }
                }

                // Debug logging
                if (iter % 100 == 0)
                {
                    System.Diagnostics.Debug.WriteLine($"[PF] Iter {iter}: pos=({current.X:F1},{current.Y:F1}), dist to goal={distToGoal:F1} cells");
                }
            }

            // Selesai iterasi tapi belum mencapai threshold goal → anggap GAGAL
            float finalDist = CalcDistance(
                current,
                new PointF(goal.X + 0.5f, goal.Y + 0.5f)
            );

            System.Diagnostics.Debug.WriteLine(
                $"[PF] Max iterations reached. Final distance to goal = {finalDist:F2} cells, waypoints = {path.Count}"
            );

            // Hanya terima path kalau benar-benar berakhir di sel goal dan cukup dekat
            if (path.Count > 1 &&
                path[path.Count - 1].X == goal.X &&
                path[path.Count - 1].Y == goal.Y &&
                finalDist < GOAL_THRESHOLD * 1.5f)   // sedikit toleransi
            {
                return path;
            }

            // Kalau tidak, kembalikan null supaya GUI tahu path-nya gagal
            return null;

        }

        // === ATTRACTIVE FORCE (menuju goal) ===
        private PointF CalcAttractive(PointF current, Point goal)
        {
            float dx = goal.X + 0.5f - current.X;
            float dy = goal.Y + 0.5f - current.Y;
            float distToGoal = (float)Math.Sqrt(dx * dx + dy * dy);

            // Tingkatkan attractive force SANGAT AGRESIF saat dekat dengan goal
            float attractiveMultiplier = ATTRACTIVE_GAIN;

            if (distToGoal < 3.0f) // Sangat dekat (dalam 3 cells)
            {
                attractiveMultiplier *= 5.0f; // 5x lebih kuat!!
            }
            else if (distToGoal < 6.0f) // Cukup dekat (dalam 6 cells)
            {
                attractiveMultiplier *= 3.0f; // 3x lebih kuat
            }
            else if (distToGoal < 10.0f) // Mendekati (dalam 10 cells)
            {
                attractiveMultiplier *= 1.5f; // 1.5x lebih kuat
            }

            // Linear attraction
            return new PointF(attractiveMultiplier * dx, attractiveMultiplier * dy);
        }

        // === REPULSIVE FORCE (menjauhi obstacles) ===
        private PointF CalcRepulsive(PointF current, List<PointF> obstacles, float multiplier = 1.0f)
        {
            PointF total = new PointF(0, 0);

            // Cek jarak ke goal untuk mengurangi repulsive saat sudah dekat
            // (Asumsi: goal ada di sekitar obstacles, perlu "terobos" dengan hati-hati)
            // Ini perlu goal position, tapi kita tidak punya di sini
            // Jadi kita pakai multiplier dari luar

            foreach (PointF obs in obstacles)
            {
                float dx = current.X - (obs.X + 0.5f);
                float dy = current.Y - (obs.Y + 0.5f);
                float dist = (float)Math.Sqrt(dx * dx + dy * dy);

                // Hanya pengaruhi jika dalam range
                if (dist < OBSTACLE_INFLUENCE_RANGE && dist > 0.1f)
                {
                    // Inverse square law: F = k * (1/d - 1/d0) / d^2
                    float repMag = REPULSIVE_GAIN * multiplier *
                        (1.0f / dist - 1.0f / OBSTACLE_INFLUENCE_RANGE) / (dist * dist);

                    total.X += repMag * (dx / dist);
                    total.Y += repMag * (dy / dist);
                }
            }

            return total;
        }

        private float CalcDistance(PointF a, PointF b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        private bool IsValid(Point p)
        {
            return p.X >= 0 && p.X < cols && p.Y >= 0 && p.Y < rows;
        }

        // === SMOOTH PATH - Removes redundant waypoints ===
        public List<PointF> SmoothPath(List<Point> rawPath, List<PointF> obstacles)
        {
            if (rawPath == null || rawPath.Count <= 2)
            {
                // Convert Point to PointF
                List<PointF> result = new List<PointF>();
                if (rawPath != null)
                {
                    foreach (Point p in rawPath)
                    {
                        result.Add(new PointF(p.X * cellSizeCm + cellSizeCm / 2,
                                             p.Y * cellSizeCm + cellSizeCm / 2));
                    }
                }
                return result;
            }

            List<Point> smoothed = new List<Point>();
            smoothed.Add(rawPath[0]); // Tambahkan start

            int i = 0;
            while (i < rawPath.Count - 1)
            {
                // Cari waypoint terjauh yang masih bisa dicapai dengan garis lurus
                int farthest = i + 1;

                for (int j = rawPath.Count - 1; j > i + 1; j--)
                {
                    if (HasLineOfSight(rawPath[i], rawPath[j]))
                    {
                        farthest = j;
                        break;
                    }
                }

                smoothed.Add(rawPath[farthest]);
                i = farthest;
            }

            System.Diagnostics.Debug.WriteLine($"[PF] Path smoothed: {rawPath.Count} -> {smoothed.Count} waypoints");

            // Convert smoothed Point path to PointF in cm
            List<PointF> smoothedCm = new List<PointF>();
            foreach (Point p in smoothed)
            {
                // Convert grid coordinates to cm (center of cell)
                float xCm = p.X * cellSizeCm + cellSizeCm / 2;
                float yCm = p.Y * cellSizeCm + cellSizeCm / 2;
                smoothedCm.Add(new PointF(xCm, yCm));
            }

            return smoothedCm;
        }

        // Cek apakah ada line of sight (tidak ada obstacle di antara 2 titik)
        private bool HasLineOfSight(Point from, Point to)
        {
            int dx = Math.Abs(to.X - from.X);
            int dy = Math.Abs(to.Y - from.Y);
            int x = from.X;
            int y = from.Y;
            int n = 1 + dx + dy;
            int x_inc = (to.X > from.X) ? 1 : -1;
            int y_inc = (to.Y > from.Y) ? 1 : -1;
            int error = dx - dy;
            dx *= 2;
            dy *= 2;

            for (; n > 0; --n)
            {
                if (IsValid(new Point(x, y)) && grid[x, y] == CellType.Obstacle)
                    return false; // Ada obstacle di jalur

                if (error > 0)
                {
                    x += x_inc;
                    error -= dy;
                }
                else
                {
                    y += y_inc;
                    error += dx;
                }
            }

            return true; // Line of sight clear
        }
    }
}