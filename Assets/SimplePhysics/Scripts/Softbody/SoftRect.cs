using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class SoftRect : MonoBehaviour
{
    public List<Transform> points = new List<Transform>();

    public int xCount;
    public int yCount;
    public int zCount;

    public Material lineMaterial;

    [Header("Visibility")]
    public bool showRBs = true;
    public bool showLineRenderers = true;
    public bool showMesh = true;

    [Header("Mesh")]
    public Material surfaceMaterial;

    private LineRenderer[] xLines; // for each (y,z) -> line over x
    private LineRenderer[] yLines; // for each (x,z) -> line over y
    private LineRenderer[] zLines; // for each (x,y) -> line over z

    private Mesh mesh;
    private Vector3[] vertices;
    private Vector2[] uvs;
    private int[] triangles;

    // We will build a surface mesh with duplicated vertices per face (simpler, correct normals/UVs).
    private const int FaceCount = 6;

    public void Init(List<Transform> pts, int x, int y, int z, Material lineMat, Material surfaceMat = null)
    {
        points = pts;
        xCount = x;
        yCount = y;
        zCount = z;
        lineMaterial = lineMat;
        surfaceMaterial = surfaceMat;
    }

    private void Awake()
    {
        if(!showRBs)
            ApplyShowRBs(false);

        if (showLineRenderers)
            CreateLineRenderers();

        if (showMesh)
        {
            var mr = GetComponent<MeshRenderer>();
            if (surfaceMaterial != null)
                mr.sharedMaterial = surfaceMaterial;

            CreateSurfaceMesh();
            UpdateSurfaceMeshVertices();
        }
    }

    private void LateUpdate()
    {
        if (showLineRenderers)
            UpdateLineRenderers();

        if (showMesh)
            UpdateSurfaceMeshVertices();
    }

    private void ApplyShowRBs(bool visible)
    {
        if (points == null || points.Count == 0) return;

        for (int i = 0; i < points.Count; i++)
        {
            var t = points[i];
            if (t == null) continue;

            var mr = t.GetComponent<MeshRenderer>();
            if (mr != null)
                mr.enabled = visible;
        }
    }


    // -------------------- Lines --------------------

    private void CreateLineRenderers()
    {
        // X direction lines (vary x, fixed y/z)
        xLines = new LineRenderer[yCount * zCount];
        for (int y = 0; y < yCount; y++)
        {
            for (int z = 0; z < zCount; z++)
            {
                // Only create lines on the hull
                if (y != 0 && y != yCount - 1 &&
                    z != 0 && z != zCount - 1)
                    continue;

                int idx = YZIndex(y, z);
                xLines[idx] = CreateLineRenderer($"XLine_y{y}_z{z}", xCount);
            }
        }

        // Y direction lines (vary y, fixed x/z)
        yLines = new LineRenderer[xCount * zCount];
        for (int x = 0; x < xCount; x++)
        {
            for (int z = 0; z < zCount; z++)
            {
                if (x != 0 && x != xCount - 1 &&
                    z != 0 && z != zCount - 1)
                    continue;

                int idx = XZIndex(x, z);
                yLines[idx] = CreateLineRenderer($"YLine_x{x}_z{z}", yCount);
            }
        }

        // Z direction lines (vary z, fixed x/y)
        zLines = new LineRenderer[xCount * yCount];
        for (int x = 0; x < xCount; x++)
        {
            for (int y = 0; y < yCount; y++)
            {
                if (x != 0 && x != xCount - 1 &&
                    y != 0 && y != yCount - 1)
                    continue;

                int idx = XYIndex(x, y);
                zLines[idx] = CreateLineRenderer($"ZLine_x{x}_y{y}", zCount);
            }
        }
    }
    private LineRenderer CreateLineRenderer(string name, int count)
    {
        var go = new GameObject(name);
        go.transform.SetParent(transform, false);

        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;
        lr.positionCount = count;
        lr.material = lineMaterial;
        lr.startWidth = 0.1f;
        lr.endWidth = 0.1f;
        lr.generateLightingData = true;

        return lr;
    }
    private void UpdateLineRenderers()
    {
        if (points == null || points.Count == 0) return;
        if (points.Count != xCount * yCount * zCount) return;

        // X direction
        for (int y = 0; y < yCount; y++)
        {
            for (int z = 0; z < zCount; z++)
            {
                var lr = xLines[YZIndex(y, z)];
                if (lr == null) continue;

                for (int x = 0; x < xCount; x++)
                    lr.SetPosition(x, points[FlatIndex(x, y, z)].position);
            }
        }

        // Y direction
        for (int x = 0; x < xCount; x++)
        {
            for (int z = 0; z < zCount; z++)
            {
                var lr = yLines[XZIndex(x, z)];
                if (lr == null) continue;

                for (int y = 0; y < yCount; y++)
                    lr.SetPosition(y, points[FlatIndex(x, y, z)].position);
            }
        }

        // Z direction
        for (int x = 0; x < xCount; x++)
        {
            for (int y = 0; y < yCount; y++)
            {
                var lr = zLines[XYIndex(x, y)];
                if (lr == null) continue;

                for (int z = 0; z < zCount; z++)
                    lr.SetPosition(z, points[FlatIndex(x, y, z)].position);
            }
        }
    }

    // -------------------- Surface Mesh --------------------

    private void CreateSurfaceMesh()
    {
        if (xCount < 2 || yCount < 2 || zCount < 2)
        {
            Debug.LogWarning("SoftRect surface mesh needs at least 2 in each dimension.");
            return;
        }

        mesh = new Mesh();
        mesh.name = "SoftRectSurface";

        // Each face uses its own vertex grid (duplicated vertices).
        int frontBackVerts = 2 * (xCount * yCount);
        int leftRightVerts = 2 * (zCount * yCount);
        int topBottomVerts = 2 * (xCount * zCount);
        int vCount = frontBackVerts + leftRightVerts + topBottomVerts;

        vertices = new Vector3[vCount];
        uvs = new Vector2[vCount];

        // Triangles: each face has (w-1)*(h-1) quads * 2 triangles * 3 indices
        int frontBackTris = 2 * (xCount - 1) * (yCount - 1) * 6;
        int leftRightTris = 2 * (zCount - 1) * (yCount - 1) * 6;
        int topBottomTris = 2 * (xCount - 1) * (zCount - 1) * 6;
        triangles = new int[frontBackTris + leftRightTris + topBottomTris];

        int vBase = 0;
        int t = 0;

        // Front (z = 0) and Back (z = zCount-1): grid (x,y)
        BuildFace_XY(ref vBase, ref t, zFixed: 0, outwardNormalPositiveZ: false);              // front
        BuildFace_XY(ref vBase, ref t, zFixed: zCount - 1, outwardNormalPositiveZ: true);     // back

        // Left (x = 0) and Right (x = xCount-1): grid (z,y)
        BuildFace_ZY(ref vBase, ref t, xFixed: 0, outwardNormalPositiveX: false);             // left
        BuildFace_ZY(ref vBase, ref t, xFixed: xCount - 1, outwardNormalPositiveX: true);     // right

        // Top (y = 0) and Bottom (y = yCount-1): grid (x,z)
        BuildFace_XZ(ref vBase, ref t, yFixed: 0, outwardNormalPositiveY: true);              // top (remember y axis is inverted in your placement, but face is still y=0 slice)
        BuildFace_XZ(ref vBase, ref t, yFixed: yCount - 1, outwardNormalPositiveY: false);    // bottom

        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        mesh.MarkDynamic();

        var mf = GetComponent<MeshFilter>();
        mf.sharedMesh = mesh;
    }

    private void UpdateSurfaceMeshVertices()
    {
        if (!showMesh) return;
        if (mesh == null || vertices == null) return;
        if (points == null || points.Count != xCount * yCount * zCount) return;

        int v = 0;

        // Front (z=0): x/y
        for (int y = 0; y < yCount; y++)
            for (int x = 0; x < xCount; x++)
                vertices[v++] = transform.InverseTransformPoint(points[FlatIndex(x, y, 0)].position);

        // Back (z=zCount-1): x/y
        for (int y = 0; y < yCount; y++)
            for (int x = 0; x < xCount; x++)
                vertices[v++] = transform.InverseTransformPoint(points[FlatIndex(x, y, zCount - 1)].position);

        // Left (x=0): z/y
        for (int y = 0; y < yCount; y++)
            for (int z = 0; z < zCount; z++)
                vertices[v++] = transform.InverseTransformPoint(points[FlatIndex(0, y, z)].position);

        // Right (x=xCount-1): z/y
        for (int y = 0; y < yCount; y++)
            for (int z = 0; z < zCount; z++)
                vertices[v++] = transform.InverseTransformPoint(points[FlatIndex(xCount - 1, y, z)].position);

        // Top (y=0): x/z
        for (int z = 0; z < zCount; z++)
            for (int x = 0; x < xCount; x++)
                vertices[v++] = transform.InverseTransformPoint(points[FlatIndex(x, 0, z)].position);

        // Bottom (y=yCount-1): x/z
        for (int z = 0; z < zCount; z++)
            for (int x = 0; x < xCount; x++)
                vertices[v++] = transform.InverseTransformPoint(points[FlatIndex(x, yCount - 1, z)].position);

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    // Face builder: XY grid at fixed z
    private void BuildFace_XY(ref int vBase, ref int t, int zFixed, bool outwardNormalPositiveZ)
    {
        int w = xCount;
        int h = yCount;

        // UVs: u = x/(w-1), v = y/(h-1)
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                uvs[vBase + y * w + x] = new Vector2(
                    (w <= 1) ? 0f : (float)x / (w - 1),
                    (h <= 1) ? 0f : (float)y / (h - 1)
                );
            }
        }

        // Triangles (winding depends on which side is outward)
        for (int y = 0; y < h - 1; y++)
        {
            for (int x = 0; x < w - 1; x++)
            {
                int i0 = vBase + y * w + x;
                int i1 = vBase + y * w + (x + 1);
                int i2 = vBase + (y + 1) * w + x;
                int i3 = vBase + (y + 1) * w + (x + 1);

                if (outwardNormalPositiveZ)
                {
                    triangles[t++] = i0; triangles[t++] = i2; triangles[t++] = i1;
                    triangles[t++] = i1; triangles[t++] = i2; triangles[t++] = i3;
                }
                else
                {
                    triangles[t++] = i0; triangles[t++] = i1; triangles[t++] = i2;
                    triangles[t++] = i1; triangles[t++] = i3; triangles[t++] = i2;
                }
            }
        }

        vBase += w * h;
    }

    // Face builder: ZY grid at fixed x
    private void BuildFace_ZY(ref int vBase, ref int t, int xFixed, bool outwardNormalPositiveX)
    {
        int w = zCount;
        int h = yCount;

        for (int y = 0; y < h; y++)
        {
            for (int z = 0; z < w; z++)
            {
                uvs[vBase + y * w + z] = new Vector2(
                    (w <= 1) ? 0f : (float)z / (w - 1),
                    (h <= 1) ? 0f : (float)y / (h - 1)
                );
            }
        }

        for (int y = 0; y < h - 1; y++)
        {
            for (int z = 0; z < w - 1; z++)
            {
                int i0 = vBase + y * w + z;
                int i1 = vBase + y * w + (z + 1);
                int i2 = vBase + (y + 1) * w + z;
                int i3 = vBase + (y + 1) * w + (z + 1);

                if (outwardNormalPositiveX)
                {
                    triangles[t++] = i0; triangles[t++] = i2; triangles[t++] = i1;
                    triangles[t++] = i1; triangles[t++] = i2; triangles[t++] = i3;
                }
                else
                {
                    triangles[t++] = i0; triangles[t++] = i1; triangles[t++] = i2;
                    triangles[t++] = i1; triangles[t++] = i3; triangles[t++] = i2;
                }
            }
        }

        vBase += w * h;
    }

    // Face builder: XZ grid at fixed y
    private void BuildFace_XZ(ref int vBase, ref int t, int yFixed, bool outwardNormalPositiveY)
    {
        int w = xCount;
        int h = zCount;

        for (int z = 0; z < h; z++)
        {
            for (int x = 0; x < w; x++)
            {
                uvs[vBase + z * w + x] = new Vector2(
                    (w <= 1) ? 0f : (float)x / (w - 1),
                    (h <= 1) ? 0f : (float)z / (h - 1)
                );
            }
        }

        for (int z = 0; z < h - 1; z++)
        {
            for (int x = 0; x < w - 1; x++)
            {
                int i0 = vBase + z * w + x;
                int i1 = vBase + z * w + (x + 1);
                int i2 = vBase + (z + 1) * w + x;
                int i3 = vBase + (z + 1) * w + (x + 1);

                if (outwardNormalPositiveY)
                {
                    triangles[t++] = i0; triangles[t++] = i2; triangles[t++] = i1;
                    triangles[t++] = i1; triangles[t++] = i2; triangles[t++] = i3;
                }
                else
                {
                    triangles[t++] = i0; triangles[t++] = i1; triangles[t++] = i2;
                    triangles[t++] = i1; triangles[t++] = i3; triangles[t++] = i2;
                }
            }
        }

        vBase += w * h;
    }

    // -------------------- Index helpers --------------------

    private int FlatIndex(int x, int y, int z)
    {
        return (y * zCount + z) * xCount + x;
    }

    private int YZIndex(int y, int z) => y * zCount + z;
    private int XZIndex(int x, int z) => x * zCount + z;
    private int XYIndex(int x, int y) => x * yCount + y;
}
