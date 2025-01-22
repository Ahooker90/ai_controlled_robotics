using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;  // Import the geometry message types

public class PublishTransform : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "transform_topic";  // Topic to publish the transform data

    // Publish the message every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TransformMsg>(topicName);  // Register the publisher with TransformMsg
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Adjust axes: Unity (X, Y, Z) to ROS (X, Z, Y)
            TransformMsg transformMessage = new TransformMsg
            {
                translation = new Vector3Msg
                {
                    x = transform.position.x,
                    y = transform.position.z,  // Unity Z -> ROS Y
                    z = transform.position.y   // Unity Y -> ROS Z
                },
                rotation = new QuaternionMsg
                {
                    x = -transform.rotation.x,  // Adjust for coordinate system differences
                    y = -transform.rotation.z,
                    z = -transform.rotation.y,
                    w = transform.rotation.w
                }
            };

            // Publish the message to the specified topic
            ros.Publish(topicName, transformMessage);

            timeElapsed = 0;
        }
    }
}

