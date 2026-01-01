using System.Collections.Generic;
using UnityEngine;

public class SoftRectGenerator : MonoBehaviour
{
    public GameObject spherePrefab;

    [Header("Grid Dimensions (counts)")]
    public int xCount = 10;   // X dimension
    public int yCount = 6;    // Y dimension
    public int zCount = 10;   // Z dimension

    [Header("Spacing")]
    public float spacing = 0.25f;

    [Header("Joint Drives - Linear")]
    public float linearStrength = 2000f;
    public float linearDamping = 80f;
    public float linearMaxForce = 1e9f;

    [Header("Joint Drives - Angular (Slerp)")]
    public float angularStrength = 200f;
    public float angularDamping = 10f;
    public float angularMaxForce = 1e9f;

    [Header("Joint Options")]
    public bool useProjection = true;
    public float projectionDistance = 0.01f;
    public float projectionAngle = 1f;


    [ContextMenu("Generate SoftRect")]
    public void GenerateSoftRect()
    {
        if (!spherePrefab)
        {
            Debug.LogError("Sphere Prefab not assigned");
            return;
        }

        if (xCount <= 0 || yCount <= 0 || zCount <= 0)
        {
            Debug.LogError("xCount, yCount and zCount must be greater than zero");
            return;
        }

        var parent = new GameObject("SoftRect");
        parent.transform.SetParent(transform, false);

        var all = new List<Transform>(xCount * yCount * zCount);

        Rigidbody[,,] bodies = new Rigidbody[xCount, yCount, zCount];

        for (int y = 0; y < yCount; y++)
        {
            for (int z = 0; z < zCount; z++)
            {
                for (int x = 0; x < xCount; x++)
                {
                    var seg = Instantiate(spherePrefab, parent.transform);
                    seg.name = $"Seg_{x:00}_{y:00}_{z:00}";

                    // Local grid layout:
                    // X -> right
                    // -Y -> downward (hanging behavior)
                    // Z -> forward
                    seg.transform.localPosition = new Vector3(
                        x * spacing,
                        -y * spacing,
                        z * spacing
                    );
                    seg.transform.localRotation = Quaternion.identity;

                    all.Add(seg.transform);

                    var rb = seg.GetComponent<Rigidbody>();
                    if (!rb)
                        rb = seg.AddComponent<Rigidbody>();

                    bodies[x, y, z] = rb;

                    // Connect only to already created neighbors
                    // to avoid duplicate joints

                    if (x > 0)
                        CreateJoint(seg.gameObject, bodies[x - 1, y, z]);

                    if (y > 0)
                        CreateJoint(seg.gameObject, bodies[x, y - 1, z]);

                    if (z > 0)
                        CreateJoint(seg.gameObject, bodies[x, y, z - 1]);
                }
            }
        }

        // Example for later extension:
        // var softRect = parent.AddComponent<SoftRect>();
        // softRect.Init(all, xCount, yCount, zCount);
    }

    private void CreateJoint(GameObject obj, Rigidbody connected)
    {
        var j = obj.AddComponent<ConfigurableJoint>();
        j.connectedBody = connected;

        j.autoConfigureConnectedAnchor = true;
        j.anchor = Vector3.zero;

        j.xMotion = ConfigurableJointMotion.Free;
        j.yMotion = ConfigurableJointMotion.Free;
        j.zMotion = ConfigurableJointMotion.Free;

        j.angularXMotion = ConfigurableJointMotion.Free;
        j.angularYMotion = ConfigurableJointMotion.Free;
        j.angularZMotion = ConfigurableJointMotion.Free;

        j.enableCollision = false;

        // Projection can help reduce error but may add artifacts if too aggressive.
        if (useProjection)
        {
            j.projectionMode = JointProjectionMode.PositionAndRotation;
            j.projectionDistance = projectionDistance;
            j.projectionAngle = projectionAngle;
        }
        else
        {
            j.projectionMode = JointProjectionMode.None;
        }

        // --- Linear drive (position) ---
        JointDrive linDrive = new JointDrive
        {
            positionSpring = linearStrength,
            positionDamper = linearDamping,
            maximumForce = linearMaxForce
        };

        j.xDrive = linDrive;
        j.yDrive = linDrive;
        j.zDrive = linDrive;

        // Drive targets: try to keep the anchor at its rest configuration.
        j.targetPosition = Vector3.zero;

        // --- Angular drive using SLERP ---
        j.rotationDriveMode = RotationDriveMode.Slerp;

        JointDrive slerp = new JointDrive
        {
            positionSpring = angularStrength,
            positionDamper = angularDamping,
            maximumForce = angularMaxForce
        };

        j.slerpDrive = slerp;
        j.targetRotation = Quaternion.identity;
    }

    private void OnDrawGizmosSelected()
    {
        if (!spherePrefab) return;
        if (xCount <= 0 || yCount <= 0 || zCount <= 0) return;

        Gizmos.color = Color.cyan;

        float localRadius = spherePrefab.transform.localScale.x * 0.5f;

        for (int y = 0; y < yCount; y++)
        {
            for (int z = 0; z < zCount; z++)
            {
                for (int x = 0; x < xCount; x++)
                {
                    Vector3 localPos = new Vector3(x * spacing, -y * spacing, z * spacing);
                    Vector3 worldPos = transform.TransformPoint(localPos);

                    Gizmos.DrawWireSphere(worldPos, localRadius);

                    if (x > 0)
                    {
                        Vector3 leftLocal = new Vector3((x - 1) * spacing, -y * spacing, z * spacing);
                        Gizmos.DrawLine(transform.TransformPoint(leftLocal), worldPos);
                    }

                    if (y > 0)
                    {
                        Vector3 upLocal = new Vector3(x * spacing, -(y - 1) * spacing, z * spacing);
                        Gizmos.DrawLine(transform.TransformPoint(upLocal), worldPos);
                    }

                    if (z > 0)
                    {
                        Vector3 backLocal = new Vector3(x * spacing, -y * spacing, (z - 1) * spacing);
                        Gizmos.DrawLine(transform.TransformPoint(backLocal), worldPos);
                    }
                }
            }
        }
    }
}
