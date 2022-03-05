using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Balma.Math
{
    public static class Intersection
    {
        public static bool RayTriangle(float3 origin, float3 direction, float3 v0, float3 v1, float3 v2, out float3 baryCoordinates, out float3 intersection)
        {
            baryCoordinates = default;
            intersection = default;

            var edge1 = v1 - v0;
            var edge2 = v2 - v0;

            var h = math.cross(direction, edge2);
            var a = math.dot(edge1, h);

            if (a > -math.EPSILON && a < math.EPSILON)
                return false;

            var f = 1.0f / a;
            var s = origin - v0;
            baryCoordinates.y = f * math.dot(s, h);

            if (baryCoordinates.y < 0.0f || baryCoordinates.y > 1.0f)
                return false;

            var q = math.cross(s, edge1);
            baryCoordinates.z = f * math.dot(direction, q);

            if (baryCoordinates.z < 0.0 || baryCoordinates.y + baryCoordinates.z > 1.0)
                return false;

            var t = f * math.dot(edge2, q);

            if (!(t > math.EPSILON))
                return false;

            intersection = origin + direction * t;
            baryCoordinates.x = 1 - baryCoordinates.y - baryCoordinates.z;

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float Cross(float2 v1, float2 v2) => (v1.x * v2.y) - (v1.y * v2.x);

        //https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        public static bool SegmentSegment(float2 a0, float2 a1, float2 b0, float2 b1)
        {
            var A = a1 - a0;
            var AB0 = a1 - b0;
            var AB1 = a1 - b1;

            var B = b1 - b0;
            var BA0 = b1 - a0;
            var BA1 = b1 - a1;

            var sa0 = math.sign(Cross(A, AB0));
            var sa1 = math.sign(Cross(A, AB1));
            var sb0 = math.sign(Cross(B, BA0));
            var sb1 = math.sign(Cross(B, BA1));

            return sa0 != sa1
                   && sb0 != sb1
                   //Intersection in the tips isn't an intersection
                   && math.lengthsq(AB0) > math.EPSILON
                   && math.lengthsq(AB1) > math.EPSILON
                   && math.lengthsq(BA0) > math.EPSILON
                   && math.lengthsq(BA1) > math.EPSILON;
        }

        //https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        public static bool SegmentSegmentFull(float2 p1, float2 q1, float2 p2, float2 q2)
        {
            // Find the four orientations needed for general and
            // special cases
            int o1 = Orientation(p1, q1, p2);
            int o2 = Orientation(p1, q1, q2);
            int o3 = Orientation(p2, q2, p1);
            int o4 = Orientation(p2, q2, q1);

            // General case
            if (o1 != o2 && o3 != o4)
                return true;

            // Special Cases
            // p1, q1 and p2 are colinear and p2 lies on segment p1q1
            if (o1 == 0 && OnSegment(p1, p2, q1)) return true;

            // p1, q1 and q2 are colinear and q2 lies on segment p1q1
            if (o2 == 0 && OnSegment(p1, q2, q1)) return true;

            // p2, q2 and p1 are colinear and p1 lies on segment p2q2
            if (o3 == 0 && OnSegment(p2, p1, q2)) return true;

            // p2, q2 and q1 are colinear and q1 lies on segment p2q2
            if (o4 == 0 && OnSegment(p2, q1, q2)) return true;

            return false; // Doesn't fall in any of the above cases
        }

        static bool OnSegment(float2 p, float2 q, float2 r)
        {
            if (q.x <= math.max(p.x, r.x) && q.x >= math.min(p.x, r.x) &&
                q.y <= math.max(p.y, r.y) && q.y >= math.min(p.y, r.y))
                return true;

            return false;
        }

        static int Orientation(float2 p, float2 q, float2 r)
        {
            // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
            // for details of below formula.
            var val = (q.y - p.y) * (r.x - q.x) -
                      (q.x - p.x) * (r.y - q.y);

            if (val == 0) return 0; // colinear

            return (val > 0) ? 1 : 2; // clock or counterclock wise
        }
    }
}
