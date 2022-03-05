using Balma.Math;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Balma.Navigation
{
    public partial struct NavMesh
    {
        public struct RayCastResult
        {
            public bool hit;
            public NavigationPoint navPoint;
        }
        
        [BurstCompile]
        public struct RayCastJob : IJob
        {
            [ReadOnly] private NavMesh navMesh;
            private float3 origin;
            private float3 direction;
            
            [WriteOnly] public NativeReference<RayCastResult> result;

            public RayCastJob(NavMesh navMesh, float3 origin, float3 direction, NativeReference<RayCastResult> result)
            {
                this.navMesh = navMesh;
                this.origin = origin;
                this.direction = direction;
                this.result = result;
            }

            public void Execute()
            {
                var minDist = float.PositiveInfinity;
                result.Value = new RayCastResult()
                {
                    hit = false,
                    navPoint = new NavigationPoint() {triangleIndex = -1, barycentricCoordinates = -1}
                };

                for (int i = 0; i < navMesh.triangles.Length; i++)
                {
                    var triangle = navMesh.triangles[i];

                    var v0 = navMesh.vertices[navMesh.edges[triangle.e0].v0];
                    var v1 = navMesh.vertices[navMesh.edges[triangle.e1].v0];
                    var v2 = navMesh.vertices[navMesh.edges[triangle.e2].v0];

                    if (Intersection.RayTriangle(origin, direction, v0, v1, v2, out var baryCoordinates, out var hit))
                    {
                        var currentDist = math.distancesq(hit, origin);
                        if (currentDist < minDist)
                        {
                            minDist = currentDist;
                            result.Value = new RayCastResult()
                            {
                                hit = true,
                                navPoint = new NavigationPoint() 
                                {
                                    triangleIndex = i,
                                    barycentricCoordinates = baryCoordinates,
                                    worldPoint = hit,
                                }
                            };
                        }
                    }
                }
            }
        }
        
        [BurstCompile]
        public struct MultiRayCastJob : IJobFor
        {
            [ReadOnly] private NavMesh navMesh;
            [ReadOnly]private NativeArray<float3> origins;
            private float3 direction;
            
            [WriteOnly] public NativeArray<RayCastResult> results;

            public MultiRayCastJob(NavMesh navMesh, NativeArray<float3> origins, float3 direction, NativeArray<RayCastResult> results)
            {
                this.navMesh = navMesh;
                this.origins = origins;
                this.direction = direction;
                this.results = results;
            }

            public void Execute(int index)
            {
                var minDist = float.PositiveInfinity;
                var result = new RayCastResult()
                {
                    hit = false,
                    navPoint = new NavigationPoint() {triangleIndex = -1, barycentricCoordinates = -1}
                };

                var origin = origins[index];

                for (int i = 0; i < navMesh.triangles.Length; i++)
                {
                    var triangle = navMesh.triangles[i];

                    var v0 = navMesh.vertices[navMesh.edges[triangle.e0].v0];
                    var v1 = navMesh.vertices[navMesh.edges[triangle.e1].v0];
                    var v2 = navMesh.vertices[navMesh.edges[triangle.e2].v0];

                    if (Intersection.RayTriangle(origin, direction, v0, v1, v2, out var baryCoordinates, out var hit))
                    {
                        var currentDist = math.distancesq(hit, origin);
                        if (currentDist < minDist)
                        {
                            minDist = currentDist;
                            result = new RayCastResult()
                            {
                                hit = true,
                                navPoint = new NavigationPoint() 
                                {
                                    triangleIndex = i,
                                    barycentricCoordinates = baryCoordinates,
                                    worldPoint = hit,
                                }
                            };
                        }
                    }
                }

                results[index] = result;
            }
        }
    }
}