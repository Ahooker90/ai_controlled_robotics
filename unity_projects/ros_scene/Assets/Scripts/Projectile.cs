using UnityEngine;

public class Projectile : MonoBehaviour
{
    public GameObject explosionEffect; // Explosion effect prefab
    public float lifespan = 30f;        // Time before the projectile is destroyed
    public int damageAmount = 10;      // Damage dealt to the player

    void Start()
    {
        // Destroy the projectile after a certain time
        Destroy(gameObject, lifespan);
    }

    void OnCollisionEnter(Collision collision)
    {
        // Get the specific object that was hit
        GameObject hitObject = collision.collider.gameObject;
        
        //Debug.Log("Hit object: " + hitObject.name + " with tag: " + hitObject.tag);

        // Check if it is the shield
        if (hitObject.CompareTag("Shield"))
        {
            // Trigger explosion effect
            if (explosionEffect != null)
            {
                Instantiate(explosionEffect, transform.position, transform.rotation);
            }

            // Destroy the projectile
            Destroy(gameObject);
            return; // Stop further processing
        }

        // Check if it is the player
        if (hitObject.CompareTag("Player"))
        {
            // Trigger explosion effect
            if (explosionEffect != null)
            {
                Instantiate(explosionEffect, transform.position, transform.rotation);
            }

            // Handle player health
            PlayerHealth playerHealth = hitObject.GetComponent<PlayerHealth>();
            if (playerHealth != null)
            {
                playerHealth.TakeDamage(damageAmount);
            }

            // Destroy the projectile
            Destroy(gameObject);
        }
    }
}
