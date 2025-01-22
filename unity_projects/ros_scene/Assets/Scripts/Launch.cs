using UnityEngine;

public class Launcher : MonoBehaviour
{
    public GameObject missilePrefab;
    public Transform launchPoint;
    public float launchForce = 500f;

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space)) // Replace with your desired input
        {
            LaunchMissile();
        }

    }

    public void LaunchMissile()
    {
        GameObject missile = Instantiate(missilePrefab, launchPoint.position, launchPoint.rotation);
        Rigidbody rb = missile.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.AddForce(launchPoint.forward * launchForce);
        }
    }
}
