using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(Test))]
class TestEditor : Editor
{
    void OnSceneGUI()
    {
        var test = (Test) target;
        
        if (!Application.isPlaying) return;
        
        var navMesh = test.navMesh;
        
        var style = new GUIStyle();
        style.normal.textColor = Color.black;
        style.fontSize = 15;
        
        if (test.drawVertexIndices)
        {
            for (var index = 0; index < navMesh.VertexCount; index++)
            {
                var vertex = navMesh.GetPosition(index);
                Handles.Label(vertex, index.ToString(), style);
            }
        }

        if (test.drawIslands)
        {
            for (var index = 0; index < navMesh.TriangleCount; index++)
            {
                var triangle = navMesh.GetTriangle(index);

                var p0 = navMesh.GetPosition(triangle.v0);
                var p1 = navMesh.GetPosition(triangle.v1);
                var p2 = navMesh.GetPosition(triangle.v2);

                var group = triangle.group;
                var island = navMesh.GetIsland(group);
    
                Handles.Label((p0+p1+p2)/3f, $"{island}({group})", style);
            }
        }
    }
}
