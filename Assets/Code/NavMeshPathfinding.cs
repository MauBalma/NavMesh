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
        public partial struct AStarJob : IJob
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
    }
}