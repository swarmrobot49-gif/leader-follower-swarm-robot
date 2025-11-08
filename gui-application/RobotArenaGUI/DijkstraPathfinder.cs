// DijkstraPathfinder.cs
using RobotArenaGUI;
using System;
using System.Collections.Generic;
using System.Drawing; // For Point
using System.Linq; // For MinBy in .NET 6+ or manual min finding

// Assuming CellType enum is accessible, e.g. public enum CellType { Free, Obstacle, Start, Goal, PathNode }
// If it's in MainForm.cs, you might need to pass it or make it public.
// For simplicity, let's assume MainForm.CellType is accessible or we redefine it locally.
// For now, let's assume we pass the gridData which uses MainForm.CellType

namespace RobotArenaGUI
{
    public static class DijkstraPathfinder
    {
        private class Node
        {
            public Point GridPosition { get; }
            public float Cost { get; set; } // Cost from start to this node
            public Point ParentGridPosition { get; set; } // Parent node in the path

            public Node(Point gridPosition, float cost, Point parentGridPosition)
            {
                GridPosition = gridPosition;
                Cost = cost;
                ParentGridPosition = parentGridPosition;
            }

            // Optional: For debugging or if used in collections that require comparison
            public override bool Equals(object obj) => obj is Node node && GridPosition.Equals(node.GridPosition);
            public override int GetHashCode() => GridPosition.GetHashCode();
        }

        // Motion model: dx, dy, cost
        private static readonly List<Tuple<int, int, float>> MotionModel = new List<Tuple<int, int, float>>
    {
        new Tuple<int, int, float>(1, 0, 1.0f),   // Right
        new Tuple<int, int, float>(0, 1, 1.0f),   // Down
        new Tuple<int, int, float>(-1, 0, 1.0f),  // Left
        new Tuple<int, int, float>(0, -1, 1.0f),  // Up
        new Tuple<int, int, float>(-1, -1, (float)Math.Sqrt(2)), // Diagonal Up-Left
        new Tuple<int, int, float>(-1, 1, (float)Math.Sqrt(2)),  // Diagonal Down-Left
        new Tuple<int, int, float>(1, -1, (float)Math.Sqrt(2)),  // Diagonal Up-Right
        new Tuple<int, int, float>(1, 1, (float)Math.Sqrt(2))   // Diagonal Down-Right
    };

        public static List<Point> FindPath(CellType[,] gridData, Point startGridPos, Point goalGridPos, int gridCols, int gridRows)
        {
            List<Node> openSet = new List<Node>();
            HashSet<Point> closedSet = new HashSet<Point>();
            Dictionary<Point, float> costs = new Dictionary<Point, float>();
            Dictionary<Point, Point> parentMap = new Dictionary<Point, Point>();

            Node startNode = new Node(startGridPos, 0.0f, startGridPos); // Parent dari start bisa start itu sendiri
            openSet.Add(startNode);
            costs[startGridPos] = 0.0f;
            parentMap[startGridPos] = new Point(-1, -1); // Indikator tidak ada parent untuk start node

            while (openSet.Any())
            {
                Node currentNode = null;
                float minCost = float.MaxValue;
                foreach (Node node_loop in openSet) // Ganti nama variabel loop agar tidak konflik
                {
                    if (node_loop.Cost < minCost)
                    {
                        minCost = node_loop.Cost;
                        currentNode = node_loop;
                    }
                }

                if (currentNode == null) break;

                openSet.Remove(currentNode);
                closedSet.Add(currentNode.GridPosition);

                if (currentNode.GridPosition.Equals(goalGridPos))
                {
                    return ReconstructPath(parentMap, goalGridPos, startGridPos);
                }

                foreach (var move in MotionModel)
                {
                    int dx = move.Item1;
                    int dy = move.Item2;
                    float moveCost = move.Item3;

                    Point neighborGridPos = new Point(currentNode.GridPosition.X + dx,
                                                      currentNode.GridPosition.Y + dy);

                    // 1. Cek Batas Grid
                    if (neighborGridPos.X < 0 || neighborGridPos.X >= gridCols ||
                        neighborGridPos.Y < 0 || neighborGridPos.Y >= gridRows)
                    {
                        continue; // Di luar batas
                    }

                    // 2. Cek Apakah Tetangga adalah Obstacle atau Sudah Dievaluasi
                    if (gridData[neighborGridPos.X, neighborGridPos.Y] == CellType.Obstacle ||
                        closedSet.Contains(neighborGridPos))
                    {
                        continue;
                    }

                    // 3. PENGECEKAN BARU: Keamanan Gerakan Diagonal (Mencegah Memotong Sudut Obstacle)
                    // Berlaku jika robot (1x1 sel) tidak boleh menyentuh sudut obstacle saat bergerak diagonal.
                    bool isDiagonalMove = (dx != 0 && dy != 0);
                    if (isDiagonalMove)
                    {
                        // Cek dua sel yang membentuk "sudut" yang akan dilewati
                        // Sel 1: (posisi saat ini X + dx, posisi saat ini Y)
                        // Sel 2: (posisi saat ini X, posisi saat ini Y + dy)
                        Point cornerCell1 = new Point(currentNode.GridPosition.X + dx, currentNode.GridPosition.Y);
                        Point cornerCell2 = new Point(currentNode.GridPosition.X, currentNode.GridPosition.Y + dy);

                        // Jika salah satu dari sel sudut ini adalah obstacle, maka gerakan diagonal ini tidak aman
                        if (gridData[cornerCell1.X, cornerCell1.Y] == CellType.Obstacle ||
                            gridData[cornerCell2.X, cornerCell2.Y] == CellType.Obstacle)
                        {
                            continue; // Gerakan diagonal diblokir oleh obstacle di sudut
                        }
                    }

                    // 4. Lanjutkan dengan logika Dijkstra standar
                    float newCostToNeighbor = currentNode.Cost + moveCost;

                    if (!costs.ContainsKey(neighborGridPos) || newCostToNeighbor < costs[neighborGridPos])
                    {
                        costs[neighborGridPos] = newCostToNeighbor;
                        parentMap[neighborGridPos] = currentNode.GridPosition;

                        Node neighborNodeInOpenSet = openSet.FirstOrDefault(n => n.GridPosition.Equals(neighborGridPos));
                        if (neighborNodeInOpenSet != null)
                        {
                            neighborNodeInOpenSet.Cost = newCostToNeighbor;
                        }
                        else
                        {
                            openSet.Add(new Node(neighborGridPos, newCostToNeighbor, currentNode.GridPosition));
                        }
                    }
                }
            }
            return null; // Path tidak ditemukan
        }

        // Fungsi ReconstructPath tetap sama, pastikan parent untuk start node dihandle dengan benar
        private static List<Point> ReconstructPath(Dictionary<Point, Point> parentMap, Point goalGridPos, Point startGridPos)
        {
            List<Point> path = new List<Point>();
            Point current = goalGridPos;

            // Cek apakah goal bisa dicapai (apakah ada di dalam map parent)
            // atau jika goal adalah start itu sendiri (path kosong).
            if (!parentMap.ContainsKey(current) && !current.Equals(startGridPos))
            {
                return null; // Path tidak ditemukan
            }

            // Lacak kembali dari GOAL ke START, tambahkan setiap node ke path.
            // Loop akan berhenti TEPAT SEBELUM menambahkan start node.
            while (!current.Equals(startGridPos))
            {
                path.Add(current);
                current = parentMap[current]; // Pindah ke parent dari node saat ini
            }

            // Karena path dibuat dari goal -> start, kita perlu membaliknya.
            path.Reverse();

            return path;
        }
    }

}
