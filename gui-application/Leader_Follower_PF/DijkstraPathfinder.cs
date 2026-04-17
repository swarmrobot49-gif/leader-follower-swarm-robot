// DijkstraPathfinder_NoDiagonalNearObstacles.cs
// SOLUSI ALTERNATIF: Disable semua gerakan diagonal jika ada obstacle di sekitar
using HybridDijkstraPotentialField;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace HybridDijkstraPotentialField
{
    public static class DijkstraPathfinderNoDiagObst
    {
        private class Node
        {
            public Point GridPosition { get; }
            public float Cost { get; set; }
            public Point ParentGridPosition { get; set; }

            public Node(Point gridPosition, float cost, Point parentGridPosition)
            {
                GridPosition = gridPosition;
                Cost = cost;
                ParentGridPosition = parentGridPosition;
            }

            public override bool Equals(object obj) => obj is Node node && GridPosition.Equals(node.GridPosition);
            public override int GetHashCode() => GridPosition.GetHashCode();
        }

        private static readonly List<Tuple<int, int, float>> MotionModel = new List<Tuple<int, int, float>>
        {
            new Tuple<int, int, float>(1, 0, 1.0f),
            new Tuple<int, int, float>(0, 1, 1.0f),
            new Tuple<int, int, float>(-1, 0, 1.0f),
            new Tuple<int, int, float>(0, -1, 1.0f),
            new Tuple<int, int, float>(-1, -1, (float)Math.Sqrt(2)),
            new Tuple<int, int, float>(-1, 1, (float)Math.Sqrt(2)),
            new Tuple<int, int, float>(1, -1, (float)Math.Sqrt(2)),
            new Tuple<int, int, float>(1, 1, (float)Math.Sqrt(2))
        };

        public static List<Point> FindPath(CellType[,] gridData, Point startGridPos, Point goalGridPos, int gridCols, int gridRows)
        {
            List<Node> openSet = new List<Node>();
            HashSet<Point> closedSet = new HashSet<Point>();
            Dictionary<Point, float> costs = new Dictionary<Point, float>();
            Dictionary<Point, Point> parentMap = new Dictionary<Point, Point>();

            Node startNode = new Node(startGridPos, 0.0f, startGridPos);
            openSet.Add(startNode);
            costs[startGridPos] = 0.0f;
            parentMap[startGridPos] = new Point(-1, -1);

            while (openSet.Any())
            {
                Node currentNode = null;
                float minCost = float.MaxValue;
                foreach (Node node_loop in openSet)
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

                    if (neighborGridPos.X < 0 || neighborGridPos.X >= gridCols ||
                        neighborGridPos.Y < 0 || neighborGridPos.Y >= gridRows)
                    {
                        continue;
                    }

                    if (gridData[neighborGridPos.X, neighborGridPos.Y] == CellType.Obstacle ||
                        closedSet.Contains(neighborGridPos))
                    {
                        continue;
                    }

                    bool isDiagonalMove = (dx != 0 && dy != 0);

                    if (isDiagonalMove)
                    {
                        // Corner cutting check
                        Point cornerCell1 = new Point(currentNode.GridPosition.X + dx, currentNode.GridPosition.Y);
                        Point cornerCell2 = new Point(currentNode.GridPosition.X, currentNode.GridPosition.Y + dy);

                        if (gridData[cornerCell1.X, cornerCell1.Y] == CellType.Obstacle ||
                            gridData[cornerCell2.X, cornerCell2.Y] == CellType.Obstacle)
                        {
                            continue;
                        }

                        // SOLUSI KUAT: Cek apakah ada obstacle di 3x3 area sekitar CURRENT atau NEIGHBOR
                        bool nearObstacle = HasObstacleInRadius(gridData, currentNode.GridPosition, gridCols, gridRows, 1) ||
                                          HasObstacleInRadius(gridData, neighborGridPos, gridCols, gridRows, 1);

                        if (nearObstacle && !neighborGridPos.Equals(goalGridPos))
                        {
                            // DISABLE diagonal completely jika dekat obstacle
                            // Kecuali jika ini adalah goal position
                            continue;
                        }
                    }

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
            return null;
        }

        // Helper: Check if there's any obstacle within given radius
        private static bool HasObstacleInRadius(CellType[,] gridData, Point center, int gridCols, int gridRows, int radius)
        {
            for (int dx = -radius; dx <= radius; dx++)
            {
                for (int dy = -radius; dy <= radius; dy++)
                {
                    if (dx == 0 && dy == 0) continue; // Skip center

                    int checkX = center.X + dx;
                    int checkY = center.Y + dy;

                    if (checkX >= 0 && checkX < gridCols && checkY >= 0 && checkY < gridRows)
                    {
                        if (gridData[checkX, checkY] == CellType.Obstacle)
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private static List<Point> ReconstructPath(Dictionary<Point, Point> parentMap, Point goalGridPos, Point startGridPos)
        {
            List<Point> path = new List<Point>();
            Point current = goalGridPos;

            if (!parentMap.ContainsKey(current) && !current.Equals(startGridPos))
            {
                return null;
            }

            while (!current.Equals(startGridPos))
            {
                path.Add(current);
                current = parentMap[current];
            }

            path.Reverse();
            return path;
        }
    }
}