using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace HybridDijkstraPotentialField
{
    /// <summary>
    /// Ramer-Douglas-Peucker Algorithm untuk path simplification
    /// Mengurangi jumlah waypoints sambil mempertahankan bentuk path
    /// </summary>
    public static class PathSimplifier
    {
        /// <summary>
        /// Simplify path using Ramer-Douglas-Peucker algorithm
        /// </summary>
        /// <param name="path">Original path dari Dijkstra</param>
        /// <param name="epsilon">Tolerance (semakin besar = semakin agresif simplify)</param>
        /// <returns>Simplified path with fewer waypoints</returns>
        public static List<Point> SimplifyRDP(List<Point> path, float epsilon)
        {
            if (path == null || path.Count < 3)
                return path;

            // RDP recursive algorithm
            return RDPRecursive(path, 0, path.Count - 1, epsilon);
        }

        /// <summary>
        /// Recursive RDP implementation
        /// </summary>
        private static List<Point> RDPRecursive(List<Point> path, int startIndex, int endIndex, float epsilon)
        {
            // Base case: only 2 points
            if (endIndex - startIndex <= 1)
            {
                return new List<Point> { path[startIndex], path[endIndex] };
            }

            // Find point with maximum perpendicular distance from line segment
            float maxDistance = 0;
            int maxIndex = startIndex;

            Point lineStart = path[startIndex];
            Point lineEnd = path[endIndex];

            for (int i = startIndex + 1; i < endIndex; i++)
            {
                float distance = PerpendicularDistance(path[i], lineStart, lineEnd);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxIndex = i;
                }
            }

            // If max distance > epsilon, split and recurse
            if (maxDistance > epsilon)
            {
                // Recursively simplify left and right segments
                List<Point> leftSegment = RDPRecursive(path, startIndex, maxIndex, epsilon);
                List<Point> rightSegment = RDPRecursive(path, maxIndex, endIndex, epsilon);

                // Combine results (remove duplicate middle point)
                List<Point> result = new List<Point>(leftSegment);
                result.AddRange(rightSegment.Skip(1)); // Skip first point of right (duplicate)
                return result;
            }
            else
            {
                // All intermediate points can be removed
                return new List<Point> { path[startIndex], path[endIndex] };
            }
        }

        /// <summary>
        /// Calculate perpendicular distance from point to line segment
        /// </summary>
        private static float PerpendicularDistance(Point point, Point lineStart, Point lineEnd)
        {
            // Vector from lineStart to lineEnd
            float dx = lineEnd.X - lineStart.X;
            float dy = lineEnd.Y - lineStart.Y;

            // If line segment is a point, return distance to that point
            if (dx == 0 && dy == 0)
            {
                return Distance(point, lineStart);
            }

            // Calculate perpendicular distance using cross product formula
            // d = |cross(P-A, B-A)| / |B-A|
            float numerator = Math.Abs(
                dy * (point.X - lineStart.X) - dx * (point.Y - lineStart.Y)
            );
            float denominator = (float)Math.Sqrt(dx * dx + dy * dy);

            return numerator / denominator;
        }

        /// <summary>
        /// Euclidean distance between two points
        /// </summary>
        private static float Distance(Point a, Point b)
        {
            float dx = b.X - a.X;
            float dy = b.Y - a.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// Simple decimation: Keep every Nth waypoint
        /// Faster than RDP but less intelligent
        /// </summary>
        public static List<Point> SimplifyDecimation(List<Point> path, int skipFactor)
        {
            if (path == null || path.Count < 3 || skipFactor < 1)
                return path;

            List<Point> simplified = new List<Point>();
            
            // Always keep start
            simplified.Add(path[0]);

            // Keep every Nth intermediate point
            for (int i = skipFactor; i < path.Count - 1; i += skipFactor)
            {
                simplified.Add(path[i]);
            }

            // Always keep goal
            if (simplified[simplified.Count - 1] != path[path.Count - 1])
            {
                simplified.Add(path[path.Count - 1]);
            }

            return simplified;
        }

        /// <summary>
        /// Adaptive simplification based on grid cell size
        /// </summary>
        public static List<Point> SimplifyAdaptive(List<Point> path, float gridCellSizeCm)
        {
            if (path == null || path.Count < 3)
                return path;

            // Determine epsilon based on grid size
            float epsilon;
            if (gridCellSizeCm <= 5.0f)
            {
                epsilon = 1.5f;  // Aggressive simplification for fine grid
            }
            else if (gridCellSizeCm <= 10.0f)
            {
                epsilon = 1.0f;  // Moderate simplification
            }
            else
            {
                epsilon = 0.5f;  // Minimal simplification for coarse grid
            }

            return SimplifyRDP(path, epsilon);
        }

        /// <summary>
        /// Calculate path metrics for comparison
        /// </summary>
        public static PathMetrics CalculateMetrics(List<Point> path, float gridCellSizeCm)
        {
            if (path == null || path.Count < 2)
                return new PathMetrics();

            float totalLength = 0;
            float totalAngleChange = 0;

            // Calculate length
            for (int i = 1; i < path.Count; i++)
            {
                float dx = (path[i].X - path[i - 1].X) * gridCellSizeCm;
                float dy = (path[i].Y - path[i - 1].Y) * gridCellSizeCm;
                totalLength += (float)Math.Sqrt(dx * dx + dy * dy);
            }

            // Calculate total angle change (smoothness metric)
            for (int i = 1; i < path.Count - 1; i++)
            {
                Point v1 = new Point(path[i].X - path[i - 1].X, path[i].Y - path[i - 1].Y);
                Point v2 = new Point(path[i + 1].X - path[i].X, path[i + 1].Y - path[i].Y);

                float dot = v1.X * v2.X + v1.Y * v2.Y;
                float mag1 = (float)Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y);
                float mag2 = (float)Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y);

                if (mag1 > 0.001f && mag2 > 0.001f)
                {
                    float cosAngle = dot / (mag1 * mag2);
                    cosAngle = Math.Max(-1.0f, Math.Min(1.0f, cosAngle));
                    totalAngleChange += (float)Math.Acos(cosAngle);
                }
            }

            return new PathMetrics
            {
                WaypointCount = path.Count,
                PathLengthCm = totalLength,
                TotalAngleChangeRad = totalAngleChange,
                SmoothnessScore = (path.Count > 2) ? totalAngleChange / (path.Count - 2) : 0
            };
        }
    }

    /// <summary>
    /// Metrics untuk evaluasi path quality
    /// </summary>
    public class PathMetrics
    {
        public int WaypointCount { get; set; }
        public float PathLengthCm { get; set; }
        public float TotalAngleChangeRad { get; set; }
        public float SmoothnessScore { get; set; }  // Lower = smoother

        public override string ToString()
        {
            return $"Waypoints: {WaypointCount}, " +
                   $"Length: {PathLengthCm:F1} cm, " +
                   $"Smoothness: {SmoothnessScore:F3} rad/wp";
        }
    }
}
