using UnityEngine;

public class EnemyLobber : MonoBehaviour
{
    public GameObject projectilePrefab; // Prefab of the sphere to lob
    public Transform player;            // Reference to the player's transform
    public Transform launchPoint;       // Point from which the sphere is launched
    public float launchForce = 10f;     // Base force for the lob
    public float launchAngle = 45f;     // Angle of the lob in degrees
    public float timeBetweenLobs = 2f;  // Time interval between lobs
    public int damageAmount = 10;       // Damage dealt to the player

    private float lobTimer;

    void Update()
    {
        lobTimer += Time.deltaTime;

        if (lobTimer >= timeBetweenLobs)
        {
            LobProjectile();
            lobTimer = 0f;
        }
    }

    void LobProjectile()
    {
        if (projectilePrefab == null || player == null || launchPoint == null)
        {
            Debug.LogWarning("Missing references in EnemyLobber!");
            return;
        }

        // Instantiate the projectile
        GameObject projectile = Instantiate(projectilePrefab, launchPoint.position, Quaternion.identity);

        // Calculate the direction and velocity for the lob
        Vector3 direction = (player.position - launchPoint.position).normalized;
        float horizontalDistance = Vector3.Distance(new Vector3(player.position.x, 0, player.position.z), new Vector3(launchPoint.position.x, 0, launchPoint.position.z));
        float verticalDistance = player.position.y - launchPoint.position.y;

        // Calculate velocity components based on angle and physics
        float gravity = Mathf.Abs(Physics.gravity.y);
        float angleInRadians = launchAngle * Mathf.Deg2Rad;

        float velocity = Mathf.Sqrt((gravity * horizontalDistance * horizontalDistance) / (2 * (horizontalDistance * Mathf.Tan(angleInRadians) - verticalDistance)));
        Vector3 velocityVector = direction * velocity * Mathf.Cos(angleInRadians) + Vector3.up * velocity * Mathf.Sin(angleInRadians);

        // Apply velocity to the projectile
        Rigidbody rb = projectile.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.velocity = velocityVector;
        }
        else
        {
            Debug.LogError("Projectile prefab must have a Rigidbody component!");
        }
    }
}

