using UnityEngine;

public class Missile : MonoBehaviour
{
    public float speed = 20f;
    public float lifeTime = 5f;
    public GameObject explosionEffect;

    void Start()
    {
        // Destroy the missile after a certain time
        Destroy(gameObject, lifeTime);
    }

    void Update()
    {
        // Ensure movement is in the forward direction of the missile
        transform.Translate(Vector3.forward * speed * Time.deltaTime, Space.Self);
    }


    void OnTriggerEnter(Collider collider)
    {
        // Check if the missile hit a target
        if (collider.gameObject.CompareTag("Enemy"))
        {
            // Trigger an explosion effect
            if (explosionEffect != null)
            {
                Instantiate(explosionEffect, transform.position, transform.rotation);
            }
            Debug.Log("TARGET HIT!");
            // Destroy the enemy object
            Destroy(collider.gameObject);

            // Destroy the missile
            Destroy(gameObject);
        }
    }
}
