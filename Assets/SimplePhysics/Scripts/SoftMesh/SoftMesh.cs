using System.Threading.Tasks;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshCollider))]
public class SoftMesh : MonoBehaviour
{
    public float minImpactImpulse = 0.5f;
    public float strength = 0.05f;
    public float maxDistance = 1.0f;
    public bool updateCollider = true;
    public LayerMask deformLayers;
    public bool reset = false;

    MeshFilter mf;
    MeshCollider mc;
    Mesh mesh;

    Vector3[] originalVertices;
    Vector3[] deformedVertices;

    // worker result
    volatile bool hasResult;
    Vector3[] resultVertices;


    void Awake()
    {
        mf = GetComponent<MeshFilter>();
        mc = GetComponent<MeshCollider>();

        mesh = Instantiate(mf.sharedMesh);
        mf.sharedMesh = mesh;

        originalVertices = mesh.vertices;
        deformedVertices = (Vector3[])originalVertices.Clone();

        mc.sharedMesh = mesh;
    }
    void Update()
    {
        if (reset)
        {
            deformedVertices = (Vector3[])originalVertices.Clone();
            ApplyToMesh(deformedVertices);
            reset = false;
        }

        if (hasResult)
        {
            hasResult = false;
            deformedVertices = resultVertices;
            ApplyToMesh(deformedVertices);
        }
    }
    void OnCollisionEnter(Collision c)
    {
        if (c.contactCount == 0) return;
        if (c.impulse.magnitude < minImpactImpulse) return;
        if ((deformLayers.value & (1 << c.gameObject.layer)) == 0) return;

        var cp = c.GetContact(0);
        Vector3 impactPointWS = cp.point;
        Vector3 impactNormalWS = -cp.normal;

        // snapshot everything needed (DON'T touch transform/mesh in worker)
        Matrix4x4 l2w = transform.localToWorldMatrix;
        Matrix4x4 w2l = transform.worldToLocalMatrix;
        Vector3 nWS = impactNormalWS.normalized;

        Vector3[] baseVerts = (Vector3[])deformedVertices.Clone();
        float maxD = maxDistance;
        float str = strength;


        Task.Run(() =>
        {
            for (int i = 0; i < baseVerts.Length; i++)
            {
                Vector3 vWS = l2w.MultiplyPoint3x4(baseVerts[i]);
                float dist = Vector3.Distance(vWS, impactPointWS);
                if (dist > maxD) continue;

                float t = Mathf.Clamp01(dist / maxD);
                float w = Mathf.SmoothStep(1f, 0f, t);
                Vector3 vWSDef = vWS - nWS * (str * w);

                baseVerts[i] = w2l.MultiplyPoint3x4(vWSDef);
            }

            resultVertices = baseVerts;
            hasResult = true;
        });
    }
    void ApplyToMesh(Vector3[] verts)
    {
        mesh.vertices = verts;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        if (updateCollider)
        {
            mc.sharedMesh = null;
            mc.sharedMesh = mesh;
        }
    }

}
