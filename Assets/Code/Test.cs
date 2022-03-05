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
    public bool drawBorders = true;
    public bool drawIslands = true;
    public bool drawAStar = true;
    public bool drawStartVisibleVertices = true;
    public bool calculateFlowField = true;
    public bool drawFlowFieldVertexDirection = true;
    public bool drawFlowFieldGridDirectionBarycentric = true;
    public bool drawFlowFieldGridDirectionNew = true;
    public int gridTestSize = 256;
    public bool flowSampleEndPoint = true;

    public float directionLenght = 0.01f;
    
    public NavMesh navMesh;

    private static readonly float3 drawOffset = new float3(0, 1, 0) * 0.005f;
    
    private NativeArray<float3> gridPoints;
    private NativeArray<NavMesh.RayCastResult> gridNavPoints;
    private NativeList<NavMesh.NavigationPoint> gridNavPointsReal;

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
        
        var step = 1f / (gridTestSize + 1);
        
        gridPoints = new NativeArray<float3>(gridTestSize * gridTestSize, Allocator.Persistent);
        gridNavPoints = new NativeArray<NavMesh.RayCastResult>(gridTestSize * gridTestSize, Allocator.Persistent);
        gridNavPointsReal = new NativeList<NavMesh.NavigationPoint>(gridTestSize * gridTestSize, Allocator.Persistent);
        for (int j = 0; j < gridTestSize; j++)
        {
            for (int i = 0; i < gridTestSize; i++)
            {
                gridPoints[i + j * gridTestSize] = new Vector3((1+i) * step, 0.1f, (1+j) * step);
            }
        }

        new NavMesh.MultiRayCastJob(navMesh, gridPoints, Vector3.down, gridNavPoints)
            .ScheduleParallel(gridPoints.Length, 16, default).Complete();

        for (int g = 0; g < gridNavPoints.Length; g++)
        {
            var r = gridNavPoints[g];
            if(r.hit)
                gridNavPointsReal.Add(r.navPoint);
        }
    }
    
    private void OnDestroy()
    {
        navMesh.Dispose();
        gridPoints.Dispose();
        gridNavPoints.Dispose();
        gridNavPointsReal.Dispose();
    }

    private void Update()
    {
        if(drawAStar) DoAStarPath();
        if(drawStartVisibleVertices) DrawStartVisibility();
        if(calculateFlowField) CalcuFlowField();
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
    
    private void CalcuFlowField()
    {
        using var startResult = new NativeReference<NavMesh.RayCastResult>(Allocator.TempJob);

        var startJobHandle = new NavMesh.RayCastJob(navMesh, startHandle.position, Vector3.down, startResult).Schedule();
        startJobHandle.Complete();

        if (startResult.Value.hit)
        {
            Debug.DrawLine(startResult.Value.navPoint.worldPoint, startHandle.position, Color.green, 0, true);
            DebugExtension.DebugWireSphere(startResult.Value.navPoint.worldPoint, Color.green, 0.01f, 0, true);

            var field = new NativeArray<NavMesh.FlowFieldNode>(navMesh.VertexCount, Allocator.TempJob);
            new NavMesh.ParentFieldJob(navMesh, startResult.Value.navPoint.triangleIndex, startResult.Value.navPoint.worldPoint, field).Run();

            if (drawFlowFieldVertexDirection)
            {
                for (int i = 0; i < field.Length; i++)
                {
                    var node = field[i];
                
                    if(!node.valid) continue;
                
                    var p0 = navMesh.GetPosition(i);
                    var p1 = node.vParent < navMesh.VertexCount ? navMesh.GetPosition(node.vParent) : startResult.Value.navPoint.worldPoint;
                
                    //Debug.DrawLine(p0 + drawOffset, p0 + math.normalize(p1-p0) * directionLenght + drawOffset, Color.red, 0, true);
                    DebugExtension.DebugArrow(p0 + drawOffset, math.normalize(p1-p0) * directionLenght, Color.red, 0, true);
                }
            }
            
            if (drawFlowFieldGridDirectionNew)
            {
                var directions = new NativeArray<float3>(gridNavPointsReal.Length, Allocator.TempJob);
                new NavMesh.SampleFieldDirectionJob(navMesh, field, gridNavPointsReal, directions)
                    .ScheduleParallel(gridNavPointsReal.Length, 16, default).Complete();

                for (int i = 0; i < gridNavPointsReal.Length; i++)
                {
                    DebugExtension.DebugArrow(gridNavPointsReal[i].worldPoint + drawOffset,  directions[i] * directionLenght, Color.red, 0, true);
                }

                directions.Dispose();
            }

            if (drawFlowFieldGridDirectionBarycentric)
            {
                for (int k = 0; k < gridPoints.Length; k++)
                {
                    var result = gridNavPoints[k];

                    if (result.hit)
                    {
                        //DebugExtension.DebugPoint(result.navPoint.worldPoint, Color.blue, 0.0075f, 0f);
                        var triangle = navMesh.GetTriangle(result.navPoint.triangleIndex);

                        float3 GetDir(int vertexIndex)
                        {
                            var n = field[vertexIndex];
                            var p = navMesh.GetPosition(vertexIndex);
                            var pp = n.vParent < navMesh.VertexCount ? navMesh.GetPosition(n.vParent) : startResult.Value.navPoint.worldPoint;
                            var d = math.normalize(pp - p);
                            return d;
                        }

                        var d0 = GetDir(triangle.v0);
                        var d1 = GetDir(triangle.v1);
                        var d2 = GetDir(triangle.v2);

                        var dir = (d0 * result.navPoint.barycentricCoordinates[0]
                                   + d1 * result.navPoint.barycentricCoordinates[1]
                                   + d2 * result.navPoint.barycentricCoordinates[2]);


                        DebugExtension.DebugArrow(result.navPoint.worldPoint + drawOffset, math.normalize(dir) * directionLenght, Color.red, 0, true);
                    }
                }
            }

            if (flowSampleEndPoint)
            {
                using var endResult = new NativeReference<NavMesh.RayCastResult>(Allocator.TempJob);
                new NavMesh.RayCastJob(navMesh, endHandle.position, Vector3.down, endResult).Run();
                
                if (endResult.Value.hit)
                {
                    Debug.DrawLine(endResult.Value.navPoint.worldPoint, endHandle.position, Color.green, 0, true);
                    DebugExtension.DebugWireSphere(endResult.Value.navPoint.worldPoint, Color.green, 0.01f, 0, true);

                    var dir = navMesh.SampleFieldDirection(field, endResult.Value.navPoint);
                    DebugExtension.DebugArrow(endResult.Value.navPoint.worldPoint + drawOffset, dir /** directionLenght * 3*/, Color.red, 0, true);
                }
                else
                {
                    Debug.DrawLine(endHandle.position, endHandle.position + Vector3.down * 1000, Color.red, 0, true);
                }
            }

            field.Dispose();
        }
        else
        {
            Debug.DrawLine(startHandle.position, startHandle.position + Vector3.down * 1000, Color.red, 0, true);
        }
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
                
                if(!navMesh.GetEdge(triangle.e0).IsBorder) Debug.DrawLine(p0, p1, new Color(1,1,1,0.05f), 0, true);
                if(!navMesh.GetEdge(triangle.e1).IsBorder) Debug.DrawLine(p1, p2, new Color(1,1,1,0.05f), 0, true);
                if(!navMesh.GetEdge(triangle.e2).IsBorder) Debug.DrawLine(p2, p0, new Color(1,1,1,0.05f), 0, true);
            }
        }
        
        if (drawBorders)
        {
            for (var index = 0; index < navMesh.TriangleCount; index++)
            {
                var triangle = navMesh.GetTriangle(index);

                var p0 = navMesh.GetPosition(triangle.v0);
                var p1 = navMesh.GetPosition(triangle.v1);
                var p2 = navMesh.GetPosition(triangle.v2);
                
                if(navMesh.GetEdge(triangle.e0).IsBorder) Debug.DrawLine(p0, p1, Color.blue, 0, true);
                if(navMesh.GetEdge(triangle.e1).IsBorder) Debug.DrawLine(p1, p2, Color.blue, 0, true);
                if(navMesh.GetEdge(triangle.e2).IsBorder) Debug.DrawLine(p2, p0, Color.blue, 0, true);
            }
        }
    }
}
