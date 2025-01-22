using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Import the standard message types

public class PublishString : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "string_topic"; // Topic name for clarity

    // The string message to send
    public string messageToSend;

    // Publish the message every N seconds
    public float publishMessageFrequency = 1.0f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName); // Register the publisher with StringMsg
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Only publish if the messageToSend is "Green" or "Blue"
            if (messageToSend == "Green" || messageToSend == "Blue")
            {
                // Create a new StringMsg with the public string variable
                StringMsg stringMessage = new StringMsg(messageToSend);

                // Publish the message to the specified topic
                ros.Publish(topicName, stringMessage);

                //Debug.Log($"Published message: {messageToSend}");
            }

            // Reset the timer
            timeElapsed = 0;
        }
    }
}
