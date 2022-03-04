using System;
using System.Collections.Generic;
using System.Linq;
using Balma.Navigation;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Random = Unity.Mathematics.Random;

public class Test : MonoBehaviour
{
    [Serializable]
    public struct MeshSetting
    {
        public MeshFilter meshFilter;
        [Range(0,1f)] public float missingTriangles;
    }
    
    public Transform startHandle;
    public Transform endHandle;
    
    public MeshFilter[] meshFilters;
    public MeshSetting[] meshSettings;
    public bool drawVertexIndices = true;
    public bool drawEdges = true;
    public bool drawIslands = true;
    public bool drawAStar = true;
    public bool drawStartVisibleVertices = true;
    public bool drawRawFlowField = true;
    
    public NavMesh navMesh;

    private static readonly float3 drawOffset = new float3(0, 1, 0) * 0.005f;

    private void Start()
    {
        navMesh = new NavMesh(Allocator.Persistent, int.MaxValue);

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
        
        foreach (var meshSetting in meshSettings)
        {
            var meshFilter = meshSetting.meshFilter;
            
            if (!meshFilter.gameObject.activeSelf) continue;

            var newMesh = new Mesh();
            newMesh.vertices = meshFilter.sharedMesh.vertices.ToArray();

            var tris = new List<int>();
            var rng = new Random(420);

            for (int i = 0; i < meshFilter.sharedMesh.triangles.Length; i+=3)
            {
                if(rng.NextFloat() < meshSetting.missingTriangles) continue;
                
                tris.Add(meshFilter.sharedMesh.triangles[i]);
                tris.Add(meshFilter.sharedMesh.triangles[i+1]);
                tris.Add(meshFilter.sharedMesh.triangles[i+2]);
            }
            
            newMesh.triangles = tris.ToArray();
            newMesh.RecalculateNormals();

            meshFilter.sharedMesh = newMesh;

            var originalMesh = newMesh;
            for (int i = 0; i < originalMesh.triangles.Length; i += 3)
            {
                navMesh.AddTriangle(
                    meshFilter.transform.TransformPoint(originalMesh.vertices[originalMesh.triangles[i]]),
                    meshFilter.transform.TransformPoint(originalMesh.vertices[originalMesh.triangles[i + 1]]),
                    meshFilter.transform.TransformPoint(originalMesh.vertices[originalMesh.triangles[i + 2]]));
            }
        }
        
        navMesh.GenerateLinks();
    }

    private void Update()
    {
        if(drawAStar) DoAStarPath();
        if(drawStartVisibleVertices) DrawStartVisibility();
        if(drawRawFlowField) DrawRawFlowField();
    }

    private void DoAStarPath()
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
            new NavMesh.AStarJob(navMesh, startResult.Value.navPoint, endResult.Value.navPoint, pathResult, path).Run();

            if (pathResult.Value)
            {
                if (path.Length > 0)
                {
                    Debug.DrawLine(navMesh.GetPosition(path[0]) + drawOffset,endResult.Value.navPoint.worldPoint + drawOffset, Color.blue, 0, false);
                    for (int i = 0; i < path.Length - 1; i++)
                        Debug.DrawLine(navMesh.GetPosition(path[i]) + drawOffset,navMesh.GetPosition(path[i + 1]) + drawOffset, Color.blue, 0, false);
                    Debug.DrawLine(navMesh.GetPosition(path[path.Length - 1]) + drawOffset,startResult.Value.navPoint.worldPoint + drawOffset, Color.blue, 0, false);
                }
                else
                {
                    Debug.DrawLine(endResult.Value.navPoint.worldPoint + drawOffset,startResult.Value.navPoint.worldPoint + drawOffset, Color.blue, 0, false);
                }
            }
        }
    }
    
    private void DrawStartVisibility()
    {
        using var startResult = new NativeReference<NavMesh.RayCastResult>(Allocator.TempJob);

        var startJobHandle = new NavMesh.RayCastJob(navMesh, startHandle.position, Vector3.down, startResult).Schedule();
        startJobHandle.Complete();

        if (startResult.Value.hit)
        {
            Debug.DrawLine(startResult.Value.navPoint.worldPoint, startHandle.position, Color.green, 0, true);
            DebugExtension.DebugWireSphere(startResult.Value.navPoint.worldPoint, Color.green, 0.01f, 0, true);

            var visibleVertices = new NativeList<NavMesh.Link>(Allocator.TempJob);
            //navMesh.GenerateLinks(startResult.Value.navPoint.worldPoint, startResult.Value.navPoint.triangleIndex, ref visibleVertices, Int32.MaxValue);
            new NavMesh.GenerateLinksJob(navMesh, startResult.Value.navPoint.worldPoint, startResult.Value.navPoint.triangleIndex, Int32.MaxValue, visibleVertices).Run();

            for (int i = 0; i < visibleVertices.Length; i++)
            {
                Debug.DrawLine(startResult.Value.navPoint.worldPoint + drawOffset, navMesh.GetPosition(visibleVertices[i].vNeighbour) + drawOffset, Color.red, 0, true);
            }

            visibleVertices.Dispose();
        }
        else
        {
            Debug.DrawLine(startHandle.position, startHandle.position + Vector3.down * 1000, Color.red, 0, true);
        }
    }
    
    private void DrawRawFlowField()
    {
        using var startResult = new NativeReference<NavMesh.RayCastResult>(Allocator.TempJob);

        var startJobHandle = new NavMesh.RayCastJob(navMesh, startHandle.position, Vector3.down, startResult).Schedule();
        startJobHandle.Complete();

        if (startResult.Value.hit)
        {
            Debug.DrawLine(startResult.Value.navPoint.worldPoint, startHandle.position, Color.green, 0, true);
            DebugExtension.DebugWireSphere(startResult.Value.navPoint.worldPoint, Color.green, 0.01f, 0, true);

            var field = new NativeArray<NavMesh.FlowFieldNode>(navMesh.VertexCount, Allocator.TempJob);

            navMesh.GetParentField(startResult.Value.navPoint.triangleIndex, startResult.Value.navPoint.worldPoint, field);

            for (int i = 0; i < field.Length; i++)
            {
                var node = field[i];
                
                if(!node.valid) continue;
                
                var p0 = navMesh.GetPosition(i);
                var p1 = node.vParent < navMesh.VertexCount ? navMesh.GetPosition(node.vParent) : startResult.Value.navPoint.worldPoint;
                
                Debug.DrawLine(p0 + drawOffset, p1 + drawOffset, Color.red, 0, true);
            }

            field.Dispose();
        }
        else
        {
            Debug.DrawLine(startHandle.position, startHandle.position + Vector3.down * 1000, Color.red, 0, true);
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
                
                Debug.DrawLine(p0, p1, navMesh.GetEdge(triangle.e0).IsBorder ? Color.blue : new Color(1,1,1,0.05f), 0, true);
                Debug.DrawLine(p1, p2, navMesh.GetEdge(triangle.e1).IsBorder ? Color.blue : new Color(1,1,1,0.05f), 0, true);
                Debug.DrawLine(p2, p0, navMesh.GetEdge(triangle.e2).IsBorder ? Color.blue : new Color(1,1,1,0.05f), 0, true);
            }
        }
    }
}
