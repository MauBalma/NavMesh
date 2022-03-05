using System;
using Balma.Math;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
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

        [BurstCompile]
        public struct GenerateLinksJob : IJob
        {
            [ReadOnly] private NavMesh navMesh;
            private float3 position;
            private int triangleIndex;
            private int maxLinks;
            [WriteOnly] private NativeList<Link> result;

            public GenerateLinksJob(NavMesh navMesh, float3 position, int triangleIndex, int maxLinks, NativeList<Link> result)
            {
                this.navMesh = navMesh;
                this.position = position;
                this.triangleIndex = triangleIndex;
                this.result = result;
                this.maxLinks = maxLinks;
            }

            public void Execute()
            {
                navMesh.GenerateLinks(position, triangleIndex, ref result, maxLinks);
            }
        }

        public void GenerateLinks(float3 position, int triangleIndex, ref NativeList<Link> result, int maxLinks)
        {
            var count = 0;

            var open = new NativeList<VisibilityCone>(Allocator.Temp);
            var startTriangle = triangles[triangleIndex];

            var vertex0 = vertices[startTriangle.v0];
            var vertex1 = vertices[startTriangle.v1];
            var vertex2 = vertices[startTriangle.v2];

            result.Add(new Link(startTriangle.v0, math.distance(position, vertex0)));
            result.Add(new Link(startTriangle.v1, math.distance(position, vertex1)));
            result.Add(new Link(startTriangle.v2, math.distance(position, vertex2)));

            count += 3;

            var e0 = edges[startTriangle.e0];
            var e1 = edges[startTriangle.e1];
            var e2 = edges[startTriangle.e2];

            var cone0 = new VisibilityCone(position, vertex0, vertex1, e0);
            var cone1 = new VisibilityCone(position, vertex1, vertex2, e1);
            var cone2 = new VisibilityCone(position, vertex2, vertex0, e2);

            open.Add(cone0);
            open.Add(cone1);
            open.Add(cone2);

            while (open.Length > 0)
            {
                if (count >= maxLinks) return;

                var cone = open[open.Length - 1];
                open.RemoveAtSwapBack(open.Length - 1);

                if (cone.edge.IsBorder) continue;

                if (ProcessCone(cone, out var edge, out var candidateIndex, out var candidate, out var visibility))
                {
                    result.Add(new Link(candidateIndex, math.distance(position.xz, vertices[candidateIndex].xz)));
                    count++;
                }

                PostprocessCone(ref open, cone, visibility, candidate, edge);
            }

            open.Clear();
        }
        
        private void GenerateLinks(int vertexIndex, NativeMultiHashMap<int, Link> container, int maxLinks)
        {
            var count = 0;
            
            var open = new NativeList<VisibilityCone>(Allocator.Temp);
            var pivot = vertices[vertexIndex];

            var inEdges = vertexToEdgesIn.GetValuesForKey(vertexIndex);
            
            while (inEdges.MoveNext())
            {
                var inEdge = edges[inEdges.Current];

                if (inEdge.IsBorder)
                {
                    var conePivot = vertices[inEdge.v0];
                    container.Add(vertexIndex, new Link(inEdge.v0, math.distance(pivot.xz, conePivot.xz)));
                    count++;
                }
            }

            var outEdges = vertexToEdgesOut.GetValuesForKey(vertexIndex);
            while (outEdges.MoveNext())
            {
                var outEdge = edges[outEdges.Current];

                var conePivot = vertices[outEdge.v1];
                container.Add(vertexIndex, new Link(outEdge.v1, math.distance(pivot.xz, conePivot.xz)));
                count++;

                var edge = edges[outEdge.eNext];
                var cone = new VisibilityCone(pivot, vertices[outEdge.v1], vertices[edge.v1], edge);
                open.Add(cone);
            }
            
            while (open.Length > 0)
            {
                if(count >= maxLinks) return;
                
                var cone = open[open.Length - 1];
                open.RemoveAtSwapBack(open.Length - 1);

                if (cone.edge.IsBorder) continue;

                if (ProcessCone(cone, out var edge, out var candidateIndex, out var candidate, out var visibility))
                {
                    container.Add(vertexIndex, new Link(candidateIndex, math.distance(pivot.xz, vertices[candidateIndex].xz)));
                    count++;
                }

                PostprocessCone(ref open, cone, visibility, candidate, edge);
            }
            
            open.Clear();
        }

        private bool ProcessCone(VisibilityCone cone, out HalfEdge edge, out int candidateIndex, out float3 candidate, out PortalTest visibility)
        {
            edge = edges[cone.edge.eAdjacent];

            candidateIndex = edge.vOpposite;
            candidate = vertices[candidateIndex];

            visibility = IsPointInsidePortal(cone.pivot.xz, cone.left.xz, cone.right.xz, candidate.xz);
            return visibility == PortalTest.Inside;
        }

        private void PostprocessCone(ref NativeList<VisibilityCone> open, VisibilityCone cone, PortalTest visibility, float3 candidate, HalfEdge edge)
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
        
        public bool PointPointVisibilityIntersection(int startTriangleIndex, float3 start, float3 end)
        {
            var startTriangle = triangles[startTriangleIndex];
        
            var e0 = edges[startTriangle.e0];
            var e1 = edges[startTriangle.e1];
            var e2 = edges[startTriangle.e2];
        
            HalfEdge currentEdge;

            if (Intersection.SegmentSegmentFull(start.xz, end.xz, vertices[e0.v0].xz, vertices[e0.v1].xz))
                currentEdge = e0;
            else if (Intersection.SegmentSegmentFull(start.xz, end.xz, vertices[e1.v0].xz, vertices[e1.v1].xz))
                currentEdge = e1;
            else if (Intersection.SegmentSegmentFull(start.xz, end.xz, vertices[e2.v0].xz, vertices[e2.v1].xz))
                currentEdge = e2;
            else
                return true;
        
            while (!currentEdge.IsBorder)
            {
                var adjacent = edges[currentEdge.eAdjacent];
                
                var eA = edges[adjacent.eNext];
                var eB = edges[adjacent.ePrevious];

                if (Intersection.SegmentSegmentFull(start.xz, end.xz, vertices[eA.v0].xz, vertices[eA.v1].xz))
                    currentEdge = eA;
                else if (Intersection.SegmentSegmentFull(start.xz, end.xz, vertices[eB.v0].xz, vertices[eB.v1].xz))
                    currentEdge = eB;
                else
                    return true;
            }
        
            return false;
        }
    }
}