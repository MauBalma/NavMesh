using Balma.ADT;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Balma.Navigation
{
    public partial struct NavMesh
    {
        public struct FlowFieldNode
        {
            public bool valid;
            public int vParent;
            public float distanceToTarget;

            public FlowFieldNode(int vParent, float distanceToTarget)
            {
                this.valid = true;
                this.vParent = vParent;
                this.distanceToTarget = distanceToTarget;
            }
        }
        
        [BurstCompile]
        public struct ParentFieldJob : IJob
        {
            [ReadOnly] private NavMesh navMesh;
            private int startTriangleIndex;
            private float3 start;
            private NativeArray<FlowFieldNode> field;

            public ParentFieldJob(NavMesh navMesh, int startTriangleIndex, float3 start, NativeArray<FlowFieldNode> field)
            {
                this.navMesh = navMesh;
                this.startTriangleIndex = startTriangleIndex;
                this.start = start;
                this.field = field;
            }

            public void Execute()
            {
                navMesh.GetParentField(startTriangleIndex, start, field);
            }
        }
        
        public NativeArray<FlowFieldNode> GetParentField(int startTriangleIndex, float3 start, NativeArray<FlowFieldNode> field)
        {
            var open = new MinHeap<int>(Allocator.Temp);
            var closed = new NativeHashSet<int>(1024, Allocator.Temp);
            var costs = new NativeHashMap<int, float>(1024, Allocator.Temp);
            var initials = new NativeList<Link>(Allocator.Temp);

            GenerateLinks(start, startTriangleIndex, ref initials, maxLinks);

            for (int i = 0; i < initials.Length; i++)
            {
                var v = initials[i];
                open.Push(v.vNeighbour, v.distance);
                costs[v.vNeighbour] = v.distance;
                field[v.vNeighbour] = new FlowFieldNode(VertexCount, v.distance);
            }
            
            while (open.Count > 0)
            {
                var currentIndex = open.Pop();

                closed.Add(currentIndex);

                var currentCost = costs[currentIndex];

                var neighbours = GetLinks(currentIndex);
                while (neighbours.MoveNext())
                {
                    var neighbour = neighbours.Current;

                    if (closed.Contains(neighbour.vNeighbour)) continue;

                    var tentativeCost = currentCost + neighbour.distance;

                    if (costs.TryGetValue(neighbour.vNeighbour, out var previousCost) &&
                        tentativeCost > previousCost) continue;

                    costs[neighbour.vNeighbour] = tentativeCost;
                    open.Push(neighbour.vNeighbour, tentativeCost);
                    field[neighbour.vNeighbour] = new FlowFieldNode(currentIndex, tentativeCost);
                }
            }

            open.Dispose();
            closed.Dispose();
            costs.Dispose();
            initials.Dispose();

            return field;
        }

        public float3 SampleFieldDirection(NativeArray<FlowFieldNode> field, NavPoint point, NavPoint target, int maxLinks)
        {
            var visibles = new NativeList<Link>(Allocator.Temp);
            GenerateLinks(point.worldPoint, point.triangleIndex, ref visibles, maxLinks);

            var min = visibles[0];
            var minDist = math.distance(point.worldPoint, GetPosition(min.vNeighbour)) + field[min.vNeighbour].distanceToTarget;

            for (int i = 1; i < visibles.Length; i++)
            {
                var curr = visibles[i];
                var dist = math.distance(point.worldPoint, GetPosition(curr.vNeighbour)) + field[curr.vNeighbour].distanceToTarget;

                if (dist < minDist)
                {
                    min = curr;
                    minDist = dist;
                }
            }
            visibles.Dispose();

            if (field[min.vNeighbour].vParent == field.Length && PointPointVisibilityIntersection(point.triangleIndex, point.worldPoint, target.worldPoint))
            {
                return  math.normalize(target.worldPoint - point.worldPoint);
            }

            return math.normalize(GetPosition(min.vNeighbour) - point.worldPoint);
        }
        
        [BurstCompile]
        public struct SampleFieldDirectionJob : IJobFor
        {
            [ReadOnly] private NavMesh navMesh;
            [ReadOnly] private NativeArray<FlowFieldNode> field;
            [ReadOnly] private NavPoint target;
            [ReadOnly] private NativeArray<NavPoint> points;
            [ReadOnly] private int maxLinks;
            [WriteOnly] private NativeArray<float3> results;

            public SampleFieldDirectionJob(NavMesh navMesh, NativeArray<FlowFieldNode> field, NavPoint target, NativeArray<NavPoint> points, NativeArray<float3> results, int maxLinks)
            {
                this.navMesh = navMesh;
                this.field = field;
                this.target = target;
                this.points = points;
                this.results = results;
                this.maxLinks = maxLinks;
            }

            public void Execute(int index)
            {
                results[index] = navMesh.SampleFieldDirection(field, points[index], target, maxLinks);
            }
        }
        
        // public struct SampleFieldDirectionJobSingle : IJob
        // {
        //     [ReadOnly] private NavMesh navMesh;
        //     [ReadOnly] private NativeArray<FlowFieldNode> field;
        //     [ReadOnly] private NativeArray<NavigationPoint> points;
        //     [WriteOnly] private NativeArray<float3> results;
        //
        //     public SampleFieldDirectionJobSingle(NavMesh navMesh, NativeArray<FlowFieldNode> field, NativeArray<NavigationPoint> points, NativeArray<float3> results)
        //     {
        //         this.navMesh = navMesh;
        //         this.field = field;
        //         this.points = points;
        //         this.results = results;
        //     }
        //
        //     public void Execute()
        //     {
        //         for (int index = 0; index < points.Length; index++)
        //         {
        //             results[index] = navMesh.SampleFieldDirection(field, points[index]);
        //         }
        //     }
        // }
    }
}