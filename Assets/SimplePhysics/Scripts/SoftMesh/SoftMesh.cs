using System.Threading.Tasks;
using UnityEngine;


[RequireComponent(typeof(MeshFilter))]
public class SoftMesh : MonoBehaviour
{
    [Header("Deformation")]
    public float minImpactImpulse = 0.5f;
    public float strength = 0.05f;
    public float maxDistance = 1.0f;
    public LayerMask deformLayers;
    public bool reset = false;

    private MeshFilter meshFilter;
    private Mesh mesh;

    private Vector3[] originalVertices;
    private Vector3[] deformedVertices;

    private MeshCollider meshCollider;



    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshCollider = GetComponent<MeshCollider>();

        mesh = Instantiate(meshFilter.sharedMesh);
        mesh.name = meshFilter.sharedMesh.name + "_DeformedInstance";
        meshFilter.sharedMesh = mesh;

        originalVertices = mesh.vertices;
        deformedVertices = (Vector3[]) originalVertices.Clone();
    }
    private void Update()
    {
        if(reset)
        {
            ResetDeformation();
            reset = false;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log(collision.impulse.magnitude);

        Rigidbody rb = GetComponent<Rigidbody>();

        if (collision.contactCount == 0)
            return;

        if(collision.impulse.magnitude < minImpactImpulse)
            return;

        if ((deformLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        ContactPoint cp = collision.GetContact(0);

        Vector3 impactPointWS = cp.point;
        Vector3 impactNormalWS = -cp.normal; //flip normal to point outwards from surface
        Vector3 impactPointOffsetWS = impactPointWS + impactNormalWS * 1;

        Debug.DrawLine(impactPointWS, impactPointOffsetWS, Color.red, 2f);

        DeformAllVertices(impactPointWS, impactNormalWS);
    }

    private void DeformAllVertices(Vector3 impactPointWS, Vector3 impactNormalWS)
    {
        Vector3 nWS = impactNormalWS.normalized;

        // Cache matrices once (value copies, thread-safe)
        Matrix4x4 localToWorld = transform.localToWorldMatrix;
        Matrix4x4 worldToLocal = transform.worldToLocalMatrix;

        Vector3[] verts = deformedVertices;

        Parallel.For(0, verts.Length, i =>
        {
            Vector3 vLS = verts[i];
            Vector3 vWS = localToWorld.MultiplyPoint3x4(vLS);  // Transform vertex from local space to world space

            float dist = Vector3.Distance(vWS, impactPointWS); // Compute distance in world space
            if (dist > maxDistance)
                return;

            
            float t = Mathf.Clamp01(dist / maxDistance);            // Normalize distance to 0..1 range      
            float w = Mathf.SmoothStep(1f, 0f, t);                  // Smooth radial falloff (1 at center, 0 at edge)        
            float displacement = strength * w;                      // Compute displacement in world units          
            Vector3 vWSDeformed = vWS - nWS * displacement;         // Apply deformation along the impact normal (world space)           
            verts[i] = worldToLocal.MultiplyPoint3x4(vWSDeformed);  // Transform back from world space to local space

        });

        deformedVertices = verts;

        // Upload modified vertices to the mesh (main thread only)
        mesh.vertices = deformedVertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        if (meshCollider != null)
        {
            meshCollider.sharedMesh = null;
            meshCollider.sharedMesh = mesh;
        }
    }



    public void ResetDeformation()
    {
        for (int i = 0; i < deformedVertices.Length; i++)
            deformedVertices[i] = originalVertices[i];

        mesh.vertices = deformedVertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        if (meshCollider != null)
        {
            meshCollider.sharedMesh = null;
            meshCollider.sharedMesh = mesh;
        }
    }

}
