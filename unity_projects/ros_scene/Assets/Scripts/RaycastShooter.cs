using UnityEngine;

public class RaycastShooter : MonoBehaviour
{
    // Enum for direction selection
    public enum Direction
    {
        XDirection,
        YDirection,
        ZDirection
    }

    // Public variable to select the direction
    public Direction shootDirection;

    // Range for the raycast distance
    public float rayDistance = 10f;

    // Whether or not to visualize the ray in the Scene view
    public bool visualizeRay = true;

    // Reference to the PublishString script
    public PublishString publisher;

    void Update()
    {
        ShootRay();
    }

    void ShootRay()
    {
        // Determine the direction of the raycast based on the enum
        Vector3 direction = Vector3.zero;

        switch (shootDirection)
        {
            case Direction.XDirection:
                direction = transform.right;  // Shooting along the X-axis
                break;
            case Direction.YDirection:
                direction = transform.up;  // Shooting along the Y-axis
                break;
            case Direction.ZDirection:
                direction = transform.forward;  // Shooting along the Z-axis
                break;
        }

        // Visualize the ray in the Scene view
        if (visualizeRay)
        {
            Debug.DrawRay(transform.position, direction * rayDistance, Color.red);  // You can change the color if needed
        }

        // Perform the raycast
        RaycastHit hit;
        if (Physics.Raycast(transform.position, direction, out hit, rayDistance))
        {
            Renderer hitRenderer = hit.collider.GetComponent<Renderer>();
            if (hitRenderer != null && hitRenderer.material.color == Color.green)
            {
                // Set the message to "Shoot Target" if a green object is hit
                if (publisher != null)
                {
                    publisher.messageToSend = "Green";
                }
            }
            else if (hitRenderer != null && hitRenderer.material.color == Color.blue)
            {
                // Set the message to "Defend" if a blue object is hit
                if (publisher != null)
                {
                    publisher.messageToSend = "Blue";
                }
            }
            else
            {
                // No relevant hit; optionally clear the message
                if (publisher != null)
                {
                    publisher.messageToSend = "";
                }
            }
        }
    }
}
