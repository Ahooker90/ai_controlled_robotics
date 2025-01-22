using UnityEngine;

public class SinusoidalMover : MonoBehaviour
{
    [Header("Lateral Movement")]
    [SerializeField] private float _speed = 2.0f; // Speed of lateral movement
    [SerializeField] private float _distance = 5.0f; // Total distance to travel along the x-axis

    [Header("Sinusoidal Movement")]
    [SerializeField] private float _amplitude = 1.0f; // Amplitude of the sinusoidal wave (y-axis movement)
    [SerializeField] private float _frequency = 1.0f; // Frequency of the sinusoidal wave

    private Vector3 _startPosition; // Starting position of the object
    private float _direction = 1.0f; // Direction of movement along the x-axis

    void Start()
    {
        // Record the starting position
        _startPosition = transform.position;
    }

    void Update()
    {
        // Calculate the new x-position based on speed and direction
        float newX = transform.position.x + _direction * _speed * Time.deltaTime;

        // Check if the object reached the edges of the predefined distance
        if (Mathf.Abs(newX - _startPosition.x) >= _distance)
        {
            // Reverse direction
            _direction *= -1;
        }

        // Calculate the sinusoidal y-position based on amplitude and frequency
        float newY = _startPosition.y + Mathf.Sin(Time.time * _frequency) * _amplitude;

        // Update the object's position
        transform.position = new Vector3(newX, newY, _startPosition.z);
    }
}

