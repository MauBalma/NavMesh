using Balma.ADT;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Balma.Navigation
{
    public partial struct NavMesh
    {
        [BurstCompile]
        public struct AStarJob : IJob
        {
            [ReadOnly] public NavMesh navMesh;
            
            public NavigationPoint start;
            public NavigationPoint end;

            [WriteOnly] public NativeReference<bool> pathFound;
            //Reversed vertex path, start/end NOT included
            [WriteOnly] public NativeList<int> path;

            public AStarJob(NavMesh navMesh, NavigationPoint start, NavigationPoint end, NativeReference<bool> pathFound, NativeList<int> path)
            {
                this.navMesh = navMesh;
                this.start = start;
                this.end = end;
                this.pathFound = pathFound;
                this.path = path;
            }

            public void Execute()
            {
                var open = new DecreseableMinHeap<int>(Allocator.Temp);
                var closed = new NativeHashSet<int>(1024, Allocator.Temp);
                var costs = new NativeHashMap<int, float>(1024, Allocator.Temp);
                var parents = new NativeHashMap<int, int>(1024, Allocator.Temp);

                var endTriangle = navMesh.triangles[end.triangleIndex];
                var startTriangle = navMesh.triangles[start.triangleIndex];

                void PushInitialVertex(NavMesh navMesh, NavigationPoint start, NavigationPoint end, int vertexIndex)
                {
                    var p0 = navMesh.GetPosition(vertexIndex);
                    var distance = math.distance(start.worldPoint, p0);
                    costs[startTriangle.v0] = distance;
                    open.Push(startTriangle.v0, distance + math.distance(end.worldPoint, p0));
                }

                PushInitialVertex(navMesh, start, end, startTriangle.v0);
                PushInitialVertex(navMesh, start, end, startTriangle.v1);
                PushInitialVertex(navMesh, start, end, startTriangle.v2);
                
                while (open.Count > 0)
                {
                    var currentIndex = open.Pop();

                    closed.Add(currentIndex);

                    if (currentIndex == endTriangle.v0 || currentIndex == endTriangle.v1 || currentIndex == endTriangle.v2)
                    {
                        do path.Add(currentIndex);
                        while (parents.TryGetValue(currentIndex, out currentIndex));
                        pathFound.Value = true;
                        return;
                    }

                    var currentCost = costs[currentIndex];
                    
                    var outEdges = navMesh.vertexToEdgesOut.GetValuesForKey(currentIndex);
                    while (outEdges.MoveNext())
                    {
                        var neighbour = navMesh.edges[outEdges.Current].v1;
                        
                        if (closed.Contains(neighbour)) continue;

                        var tentativeCost = currentCost + math.distance(navMesh.vertices[neighbour], navMesh.vertices[currentIndex]);

                        if (costs.TryGetValue(neighbour, out var previousCost) && tentativeCost > previousCost) continue;

                        costs[neighbour] = tentativeCost;
                        open.Push(neighbour, tentativeCost + math.distance(end.worldPoint, navMesh.vertices[neighbour]));
                        parents[neighbour] = currentIndex;
                    }
                }

                pathFound.Value = false;
            }
        }
        
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
            var open = new DecreseableMinHeap<int>(Allocator.Temp);
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

        public float3 SampleFieldDirection(NativeArray<FlowFieldNode> field, NavigationPoint point)
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

            return math.normalize(GetPosition(min.vNeighbour) - point.worldPoint);
        }
        
        [BurstCompile]
        public struct SampleFieldDirectionJob : IJobFor
        {
            [ReadOnly] private NavMesh navMesh;
            [ReadOnly] private NativeArray<FlowFieldNode> field;
            [ReadOnly] private NativeArray<NavigationPoint> points;
            [WriteOnly] private NativeArray<float3> results;

            public SampleFieldDirectionJob(NavMesh navMesh, NativeArray<FlowFieldNode> field, NativeArray<NavigationPoint> points, NativeArray<float3> results)
            {
                this.navMesh = navMesh;
                this.field = field;
                this.points = points;
                this.results = results;
            }

            public void Execute(int index)
            {
                results[index] = navMesh.SampleFieldDirection(field, points[index]);
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