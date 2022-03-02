using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(Test))]
class TestEditor : Editor
{
    void OnSceneGUI()
    {
        var test = (Test) target;
        
        if (!Application.isPlaying) return;
        
        if (test.drawVertexIndices)
        {
            var style = new GUIStyle();
            style.normal.textColor = Color.black;
            style.fontSize = 15;
        
            for (var index = 0; index < test.navMesh.VertexCount; index++)
            {
                var vertex = test.navMesh.GetPosition(index);
                Handles.Label(vertex, index.ToString(), style);
            }
        }
    }
}
