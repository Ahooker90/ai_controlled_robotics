using UnityEngine;

public class ForceFieldSpawner : MonoBehaviour
{
    public GameObject forceFieldPrefab; // Prefab for the force field
    public float forceFieldLifetime = 5f; // Time in seconds before the force field is destroyed
    public string forceFieldLayerName = "Shield"; // Layer for the force field

    void Update()
    {
        // Check for left mouse button click
        if (Input.GetMouseButtonDown(0))
        {
            SpawnForceField();
        }
    }

    public void SpawnForceField()
    {
        // Ensure the prefab is assigned
        if (forceFieldPrefab == null)
        {
            Debug.LogWarning("Force field prefab is not assigned!");
            return;
        }

        // Spawn the force field as a child of this GameObject
        GameObject forceFieldInstance = Instantiate(forceFieldPrefab, transform.position, Quaternion.identity, transform);

        // Set the force field's layer to the specified layer
        int forceFieldLayer = LayerMask.NameToLayer(forceFieldLayerName);
        if (forceFieldLayer == -1)
        {
            Debug.LogError($"Layer '{forceFieldLayerName}' does not exist. Please create it in the Unity Editor.");
            Destroy(forceFieldInstance);
            return;
        }

        forceFieldInstance.layer = forceFieldLayer;

        // Destroy the force field after its lifetime
        Destroy(forceFieldInstance, forceFieldLifetime);
    }
}

