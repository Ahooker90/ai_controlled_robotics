using UnityEngine;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using TMPro;
using System;

public class RosSubscriber : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    public TMP_Text llmOut;
    
    // Topic name to subscribe to
    public string topicName = "/string_topic";

    // References to character components
    public ForceFieldSpawner characterForceFieldSpawner;
    public Launcher characterLauncher;

    // Cooldown-related variables
    public float commandCooldown = 1.0f; // Minimum time (in seconds) between executing commands
    private float lastCommandTime = -1f; // Tracks the last time we executed a command

    // Enum for recognized commands
    private enum Command
    {
        Shoot,
        Defend,
        Unknown
    }

    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to the specified topic
        ros.Subscribe<StringMsg>(topicName, StringMessageReceived);
    }

    // Callback function called when a message is received
    void StringMessageReceived(StringMsg message)
    {
        // Convert the message to lowercase for case-insensitive comparison
        string lowerCaseMessage = message.data.Trim().ToLower();
        Debug.Log("Received from ROS: " + lowerCaseMessage);

        if (llmOut != null)
        {
            llmOut.text = lowerCaseMessage;
        }

        Command command = ParseCommand(lowerCaseMessage);

        // Execute the command only if the cooldown has passed
        if (CanExecuteCommand())
        {
            switch (command)
            {
                case Command.Shoot:
                    if (characterLauncher != null)
                    {
                        characterLauncher.LaunchMissile();
                        UpdateLastCommandTime();
                    }
                    else
                    {
                        Debug.LogWarning("No Launcher referenced on characterLauncher!");
                    }
                    break;

                case Command.Defend:
                    if (characterForceFieldSpawner != null)
                    {
                        characterForceFieldSpawner.SpawnForceField();
                        UpdateLastCommandTime();
                    }
                    else
                    {
                        Debug.LogWarning("No ForceFieldSpawner referenced on characterForceFieldSpawner!");
                    }
                    break;

                case Command.Unknown:
                    Debug.LogWarning("Unrecognized response from ROS: " + lowerCaseMessage);
                    break;
            }
        }
        else
        {
            // If cooldown isn't over, ignore the command.
            Debug.Log("Command ignored due to cooldown.");
        }
    }

    // Helper function to parse command
    private Command ParseCommand(string message)
    {
        if (message == "<shoot>" || message == "shoot")
            return Command.Shoot;
        if (message == "<defend>"|| message == "defend")
            return Command.Defend;
        return Command.Unknown;
    }

    // Check if we can execute a command based on the cooldown
    private bool CanExecuteCommand()
    {
        return (Time.time - lastCommandTime) >= commandCooldown;
    }

    // Update the last command execution time to the current time
    private void UpdateLastCommandTime()
    {
        lastCommandTime = Time.time;
    }
}
