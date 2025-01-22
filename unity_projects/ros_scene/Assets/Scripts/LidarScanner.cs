using UnityEngine;

[ExecuteInEditMode] // This attribute ensures OnDrawGizmos is called in edit mode
public class LidarScanner : MonoBehaviour
{
    public float rayDistance = 10f;
    public float horizontalFOV = 180f; 
    public float verticalAngle = 45f;  
    public int horizontalResolution = 100; 
    public int verticalResolution = 10;    
    public bool visualizeRays = true;
    public PublishString publisher;

    void Update()
    {
        // Perform the scan only during play mode if you want to detect objects dynamically.
        // If you want scanning even in edit mode, you can move this logic as well.
        #if UNITY_EDITOR
        if (Application.isPlaying)
        #endif
        {
            PerformLidarScan();
        }
    }

    void PerformLidarScan()
    {
        float horizontalStep = horizontalFOV / (horizontalResolution - 1);
        float verticalStep = verticalAngle / (verticalResolution - 1);

        for (int v = 0; v < verticalResolution; v++)
        {
            float verticalOffset = -verticalAngle / 2 + v * verticalStep;
            for (int h = 0; h < horizontalResolution; h++)
            {
                float horizontalOffset = -horizontalFOV / 2 + h * horizontalStep;
                Vector3 direction = Quaternion.Euler(verticalOffset, horizontalOffset, 0) * transform.forward;

                // Perform the raycast
                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, rayDistance))
                {
                    Renderer hitRenderer = hit.collider.GetComponent<Renderer>();
                    if (hitRenderer != null)
                    {
                        if (hitRenderer.material.color == Color.green && publisher != null)
                        {
                            publisher.messageToSend = "Green";
                        }
                        else if (hitRenderer.material.color == Color.blue && publisher != null)
                        {
                            publisher.messageToSend = "Blue";
                        }
                    }
                }
            }
        }
    }

    void OnDrawGizmos()
    {
        if (!visualizeRays) return;

        // Only draw the rays for visualization. This is called in Edit mode as well.
        float horizontalStep = horizontalFOV / (horizontalResolution - 1);
        float verticalStep = verticalAngle / (verticalResolution - 1);

        Gizmos.color = Color.red;
        for (int v = 0; v < verticalResolution; v++)
        {
            float verticalOffset = -verticalAngle / 2 + v * verticalStep;
            for (int h = 0; h < horizontalResolution; h++)
            {
                float horizontalOffset = -horizontalFOV / 2 + h * horizontalStep;
                Vector3 direction = Quaternion.Euler(verticalOffset, horizontalOffset, 0) * transform.forward;
                Vector3 start = transform.position;
                Vector3 end = start + direction * rayDistance;

                // Draw a line representing the ray
                Gizmos.DrawLine(start, end);
            }
        }
    }
}

