using System.Collections;
using UnityEngine;

public class DifferentialDriveController : MonoBehaviour
{
    public enum ControlMode
    {
        RotationOnly,
        TranslationOnly,
        Both,
        DebugMode
    }

    [SerializeField]
    private ControlMode _controlMode = ControlMode.Both;

    [SerializeField]
    [Range(0, 20000)]
    private float _linearSpeed = 10000f; // Max forward speed
    [SerializeField]
    [Range(0, 50)]
    private float _angularSpeed = 15f; // Rotational speed factor

    [SerializeField]
    private GameObject _target = null;

    public ArticulationBody _leftWheel;
    public ArticulationBody _rightWheel;

    [Header("PID Parameters")]
    [Range(0, 5)]
    public float _angleP = 1;
    [Range(0, 100)]
    public float _angleI = 0;
    [Range(0, 100)]
    public float _angleD = 0; // PID parameters for angular control

    private PID _anglePIDController;

    void Start()
    {
        if (_leftWheel == null || _rightWheel == null)
        {
            Debug.LogError("ArticulationBodies for the wheels are missing!");
            return;
        }

        _anglePIDController = new PID(_angleP, _angleI, _angleD);

        if (_anglePIDController == null)
        {
            Debug.LogError("Failed to initialize PID Controller!");
        }
    }

    private void Update()
    {
        if (_anglePIDController != null)
        {
            _anglePIDController.Kp = _angleP;
            _anglePIDController.Ki = _angleI;
            _anglePIDController.Kd = _angleD;
        }
    }

    void FixedUpdate()
    {
        if (_target == null)
            return;

        // Transform target position into robot's local coordinates
        Vector3 localTargetPos3D = transform.InverseTransformPoint(_target.transform.position);

        // Calculate angle error; now that forward is Z, use x as "sideways" and z as "forward"
        float angleError = Mathf.Atan2(localTargetPos3D.x, localTargetPos3D.z) * Mathf.Rad2Deg;

        float angularCorrection = _anglePIDController.GetOutput(angleError, Time.fixedDeltaTime);

        // Determine if target is in front or behind (relative to Z)
        float distanceToTarget = new Vector2(localTargetPos3D.x, localTargetPos3D.z).magnitude;
        float directionSign = Mathf.Sign(localTargetPos3D.z);
        float rawSpeed = distanceToTarget * _linearSpeed * directionSign;
        float forwardSpeed = Mathf.Clamp(rawSpeed, -_linearSpeed, _linearSpeed);
        //Debug.Log($"directionSign: {directionSign}, distanceToTarget: {distanceToTarget}, rawSpeed :{rawSpeed }, forwardSpeed: {forwardSpeed}");
        
        float leftWheelVelocity = 0;
        float rightWheelVelocity = 0;

        // Adjust velocities based on the chosen control mode
        switch (_controlMode)
        {
            case ControlMode.RotationOnly:
                // Rotate in place
                leftWheelVelocity = -angularCorrection * _angularSpeed;
                rightWheelVelocity = angularCorrection * _angularSpeed;
                break;

            case ControlMode.TranslationOnly:
                // Move forward or backward depending on where the target is
                leftWheelVelocity = forwardSpeed;
                rightWheelVelocity = forwardSpeed;
                //Debug.Log($"Left Velocity: {leftWheelVelocity}, Right Velocity: {rightWheelVelocity}");
                break;

            case ControlMode.Both:
                // Blend translation and rotation
                leftWheelVelocity = forwardSpeed - angularCorrection * _angularSpeed;
                rightWheelVelocity = forwardSpeed + angularCorrection * _angularSpeed;
                break;
            case ControlMode.DebugMode:
                Debug.Log($"Angle Error: {angleError}, Angular Correction: {angularCorrection}");
                Debug.Log($"Distance: {distanceToTarget}, Forward Speed: {forwardSpeed}");
                Debug.Log($"Left Velocity: {leftWheelVelocity}, Right Velocity: {rightWheelVelocity}");
                break;
        }

        SetWheelVelocity(_leftWheel, leftWheelVelocity);
        SetWheelVelocity(_rightWheel, rightWheelVelocity);



    }

    private void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        float minVelocityThreshold = 0.0f;
        if (Mathf.Abs(velocity) < minVelocityThreshold)
        {
            velocity = Mathf.Sign(velocity) * minVelocityThreshold;
        }

        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = velocity;
        wheel.xDrive = drive;
    }
}
