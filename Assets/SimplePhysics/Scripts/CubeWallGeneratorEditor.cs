using UnityEngine;
using UnityEditor;


public class CubeWallGenerator : MonoBehaviour
{
    [Header("Dimensions")]
    public int sizeX = 5;
    public int sizeY = 5;
    public int sizeZ = 1;

    [Header("Prefab")]
    public GameObject cubePrefab;
    public float spacing = 0f;

    [ContextMenu("Generate Cubes")]
    void Generate()
    {

        if (cubePrefab == null)
        {
            Debug.LogError("Cube Prefab not assigned");
            return;
        }

        GameObject parent = new GameObject("GeneratedCubes");
        Undo.RegisterCreatedObjectUndo(parent, "Create Cube Parent");

        parent.transform.SetParent(transform, false);
        parent.transform.localPosition = Vector3.zero;
        parent.transform.localRotation = Quaternion.identity;
        parent.transform.localScale = Vector3.one;

        float step = 1f + spacing;

        for (int x = 0; x < sizeX; x++)
            for (int y = 0; y < sizeY; y++)
                for (int z = 0; z < sizeZ; z++)
                {
                    Vector3 localPos = new Vector3(x, y, z) * step;

                    GameObject cube = (GameObject)PrefabUtility.InstantiatePrefab(cubePrefab);
                    Undo.RegisterCreatedObjectUndo(cube, "Create Cube");

                    cube.transform.SetParent(parent.transform, false);
                    cube.transform.localPosition = localPos;
                    cube.transform.localRotation = Quaternion.identity;
                }
    }


    void OnDrawGizmos()
    {
        float step = 1f + spacing;

        Vector3 size = new Vector3(
            sizeX * step,
            sizeY * step,
            sizeZ * step
        );

        Vector3 centerLocal = size * 0.5f - Vector3.one * step * 0.5f;
        Vector3 centerWorld = transform.TransformPoint(centerLocal);

        Gizmos.color = Color.cyan; 
        Gizmos.matrix = transform.localToWorldMatrix;

        Gizmos.DrawWireCube(centerLocal, size);
    }
}
