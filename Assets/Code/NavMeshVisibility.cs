using Unity.Collections;
using Unity.Mathematics;

namespace Balma.Navigation
{
    public partial struct NavMesh
    {
        //TODO Jobify this functions? I need more use cases to se how to separate responsibilities

        private struct VisibilityCone
        {
            public float3 pivot;
            public float3 left;
            public float3 right;
            public HalfEdge edge;

            public VisibilityCone(float3 pivot, float3 left, float3 right, HalfEdge edge)
            {
                this.pivot = pivot;
                this.left = left;
                this.right = right;
                this.edge = edge;
            }
        }

        public struct Link
        {
            public int vNeighbour;
            public float distance;

            public Link(int vNeighbour, float distance)
            {
                this.vNeighbour = vNeighbour;
                this.distance = distance;
            }
        }

        private enum PortalTest
        {
            None,
            Left,
            Inside,
            Right,
        }

        //Should this be a job?
        public void GenerateLinks(float3 position, int triangleIndex, ref NativeList<Link> container, int maxLinks)
        {
            var count = 0;

            var conesOpen = new NativeList<VisibilityCone>(Allocator.Temp);
            var startTriangle = triangles[triangleIndex];

            var vertex0 = vertices[startTriangle.v0];
            var vertex1 = vertices[startTriangle.v1];
            var vertex2 = vertices[startTriangle.v2];

            container.Add(new Link(startTriangle.v0, math.distance(position, vertex0)));
            container.Add(new Link(startTriangle.v1, math.distance(position, vertex1)));
            container.Add(new Link(startTriangle.v2, math.distance(position, vertex2)));

            count += 3;

            var e0 = edges[startTriangle.e0];
            var e1 = edges[startTriangle.e1];
            var e2 = edges[startTriangle.e2];

            var cone0 = new VisibilityCone(position, vertex0, vertex1, e0);
            var cone1 = new VisibilityCone(position, vertex1, vertex2, e1);
            var cone2 = new VisibilityCone(position, vertex2, vertex0, e2);

            conesOpen.Add(cone0);
            conesOpen.Add(cone1);
            conesOpen.Add(cone2);

            ProcessCones(ref conesOpen, ref container, position, count, maxLinks);
            conesOpen.Clear();
        }

        private void ProcessCones(ref NativeList<VisibilityCone> open, ref NativeList<Link> container, float3 pivot, int count, int max)
        {
            while (open.Length > 0)
            {
                if (count >= max) return;

                var cone = open[open.Length - 1];
                open.RemoveAtSwapBack(open.Length - 1);

                if (cone.edge.IsBorder) continue;

                if (ProcessCone(cone, out var edge, out var candidateIndex, out var candidate, out var visibility))
                {
                    container.Add(new Link(candidateIndex, math.distance(pivot.xz, vertices[candidateIndex].xz)));
                    count++;
                }

                PostprocessCone(ref open, cone, visibility, candidate, edge);
            }
        }

        private bool ProcessCone(VisibilityCone cone, out HalfEdge edge, out int candidateIndex, out float3 candidate,
            out PortalTest visibility)
        {
            edge = edges[cone.edge.eAdjacent];

            candidateIndex = edge.vOpposite;
            candidate = vertices[candidateIndex];

            visibility = IsPointInsidePortal(cone.pivot.xz, cone.left.xz, cone.right.xz, candidate.xz);
            return visibility == PortalTest.Inside;
        }

        private void PostprocessCone(ref NativeList<VisibilityCone> open, VisibilityCone cone, PortalTest visibility,
            float3 candidate, HalfEdge edge)
        {
            var leftCone = cone;
            if (visibility != PortalTest.Left)
            {
                if (visibility == PortalTest.Inside)
                {
                    leftCone.right = candidate;
                }

                leftCone.edge = edges[edge.eNext];
                open.Add(leftCone);
            }

            var rightCone = cone;
            if (visibility != PortalTest.Right)
            {
                if (visibility == PortalTest.Inside)
                {
                    rightCone.left = candidate;
                }

                rightCone.edge = edges[edge.ePrevious];
                open.Add(rightCone);
            }
        }

        //There is no math.cross(float2,float2)
        private float Cross(float2 v1, float2 v2) => (v1.x * v2.y) - (v1.y * v2.x);

        private PortalTest IsPointInsidePortal(float2 pivot, float2 left, float2 right, float2 point)
        {
            var leftVector = left - pivot;
            var rightVector = right - pivot;
            var testVector = point - pivot;

            var a = Cross(rightVector, testVector);
            var b = Cross(testVector, leftVector);

            if (a >= 0)
            {
                if (b >= 0) return PortalTest.Inside;
                return PortalTest.Left;
            }

            return PortalTest.Right;
        }
    }
}