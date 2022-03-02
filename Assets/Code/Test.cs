using System;
using Balma.Navigation;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public class Test : MonoBehaviour
{
    public Transform startHandle;
    public Transform endHandle;
    
    public MeshFilter[] meshFilters;
    public bool drawVertexIndices = true;
    public bool drawEdges = true;
    
    public NavMesh navMesh;

    private void Start()
    {
        navMesh = new NavMesh(Allocator.Persistent);

        foreach (var meshFilter in meshFilters)
        {
            if (!meshFilter.gameObject.activeSelf) continue;

            var originalMesh = meshFilter.mesh;
            for (int i = 0; i < originalMesh.triangles.Length; i += 3)
            {
                navMesh.AddTriangle(
                    meshFilter.transform.TransformPoint(originalMesh.vertices[originalMesh.triangles[i]]),
                    meshFilter.transform.TransformPoint(originalMesh.vertices[originalMesh.triangles[i + 1]]),
                    meshFilter.transform.TransformPoint(originalMesh.vertices[originalMesh.triangles[i + 2]]));
            }
        }
    }

    private void Update()
    {
        using var startResult = new NativeReference<NavMesh.RayCastResult>(Allocator.TempJob);
        using var endResult = new NativeReference<NavMesh.RayCastResult>(Allocator.TempJob);
        
        var startJobHandle = new NavMesh.RayCastJob(navMesh, startHandle.position, Vector3.down, startResult).Schedule();
        var endJobHandle = new NavMesh.RayCastJob(navMesh, endHandle.position, Vector3.down, endResult).Schedule();
        
        JobHandle.CombineDependencies(startJobHandle, endJobHandle).Complete();

        if (startResult.Value.hit)
        {
            Debug.DrawLine(startResult.Value.navPoint.worldPoint, startHandle.position, Color.green, 0, true);
            DebugExtension.DebugWireSphere(startResult.Value.navPoint.worldPoint, Color.green, 0.01f, 0, true);
        }
        else
        {
            Debug.DrawLine(startHandle.position, startHandle.position + Vector3.down * 1000, Color.red, 0, true);
        }

        if (endResult.Value.hit)
        {
            Debug.DrawLine(endResult.Value.navPoint.worldPoint, endHandle.position, Color.green, 0, true);
            DebugExtension.DebugWireSphere(endResult.Value.navPoint.worldPoint, Color.green, 0.01f, 0, true);
        }
        else
        {
            Debug.DrawLine(endHandle.position, endHandle.position + Vector3.down * 1000, Color.red, 0, true);
        }

        if (startResult.Value.hit && endResult.Value.hit)
        {
            using var pathResult = new NativeReference<bool>(Allocator.TempJob);
            using var path = new NativeList<int>(Allocator.TempJob);
            var aStarJobHandle = new NavMesh.AStarJob(navMesh, startResult.Value.navPoint, endResult.Value.navPoint, pathResult, path).Schedule();
            
            aStarJobHandle.Complete();

            if (pathResult.Value)
            {
                var drawOffset = new float3(0,1,0) * 0.01f;
                if (path.Length > 0)
                {
                    Debug.DrawLine(navMesh.GetPosition(path[0]) + drawOffset, endResult.Value.navPoint.worldPoint + drawOffset, Color.blue, 0, false);
                    for (int i = 0; i < path.Length-1; i++)
                        Debug.DrawLine(navMesh.GetPosition(path[i]) + drawOffset, navMesh.GetPosition(path[i + 1]) + drawOffset, Color.blue, 0, false);
                    Debug.DrawLine(navMesh.GetPosition(path[path.Length-1]) + drawOffset, startResult.Value.navPoint.worldPoint + drawOffset, Color.blue, 0, false);
                }
                else
                {
                    Debug.DrawLine(endResult.Value.navPoint.worldPoint + drawOffset, startResult.Value.navPoint.worldPoint + drawOffset, Color.blue, 0, false);
                }
            }
        }
    }

    private void OnDestroy()
    {
        navMesh.Dispose();
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        
        if (drawEdges)
        {
            for (var index = 0; index < navMesh.TriangleCount; index++)
            {
                var triangle = navMesh.GetTriangle(index);

                var p0 = navMesh.GetPosition(triangle.v0);
                var p1 = navMesh.GetPosition(triangle.v1);
                var p2 = navMesh.GetPosition(triangle.v2);
                
                Debug.DrawLine(p0, p1, Color.white, 0, true);
                Debug.DrawLine(p1, p2, Color.white, 0, true);
                Debug.DrawLine(p2, p0, Color.white, 0, true);
            }
        }
    }
}
