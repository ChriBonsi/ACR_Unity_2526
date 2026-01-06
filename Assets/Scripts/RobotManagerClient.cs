using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.RobotManager;
using System.Collections.Generic;
using RosMessageTypes.Std;

public class RobotManagerClient : MonoBehaviour
{
    public GameObject robotPrefab;
    private static ROSConnection ros;
    public GameObject robotParent;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<RobotManagerRobotMsg>("robot_manager/publish_robot", SpawnRobots);

        var subscribeMsg = new StringMsg()
        {
            data = "ready"
        };

        ros.Publish("robot_manager/request_robot", subscribeMsg);
    }

    private void SpawnRobots(RobotManagerRobotMsg msg)
    {
        //Debug.Log($"[RobotManagerClient] Spawning robot {msg.robot_id} of type: {msg.robot_type} at ({msg.start_x}, {msg.start_y})");
        GameObject robotInstance = Instantiate(robotPrefab, robotParent.transform);
        Robot robot;

        switch (msg.robot_type)
        {
            case "cleaner":
                robot = robotInstance.AddComponent<CleanerRobot>();
                robotInstance.GetComponent<Renderer>().material.color = Color.blue;
                break;
            case "security":
                robot = robotInstance.AddComponent<SecurityRobot>();
                robotInstance.GetComponent<Renderer>().material.color = Color.red;
                break;
            case "baggage":
                robot = robotInstance.AddComponent<BaggageRobot>();
                robotInstance.GetComponent<Renderer>().material.color = Color.green;
                break;
            default:
                robot = robotInstance.AddComponent<Robot>();
                robotInstance.GetComponent<Renderer>().material.color = Color.gray;
                break;
        }

        robotInstance.transform.position = new Vector3(msg.start_x, msg.start_y, msg.start_z);
        robot.robotId = msg.robot_id;
        robot.endX = msg.end_x;
        robot.endY = msg.end_y;
        robot.endZ = msg.end_z;
        var path = new List<Vector3>();
        for (int i = 0; i < msg.path_x.Length; i++)
        {
            path.Add(new Vector3(msg.path_x[i], msg.path_y[i], msg.path_z[i]));
        }
        robot.destinations = path;
        robot.loop = msg.loop;
        robot.moveSpeed = msg.move_speed;
        robot.perceptionRadius = msg.perception_radius;
        robot.obstacleDistanceThreshold = msg.obstacle_distance_threshold;
        robot.robotType = msg.robot_type;
        robotInstance.name = $"{msg.robot_type}_robot_{msg.robot_id}";
        robot.currentState = RobotState.Moving;
    }

    public static void SendTrackingData(RobotManagerTrackerMsg msg)
    {
        ros.Publish("robot_manager/subscribe_tracker", msg);
        //Debug.Log($"[RobotManagerClient] Published tracking data for Robot ID: {msg.robot_id}");
    }
}
