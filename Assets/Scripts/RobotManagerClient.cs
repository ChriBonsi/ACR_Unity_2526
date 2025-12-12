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
        Robot robot = msg.robot_type switch
        {
            "cleaner" => robotInstance.AddComponent<CleanerRobot>(),
            "security" => robotInstance.AddComponent<SecurityRobot>(),
            _ => robotInstance.AddComponent<Robot>(),
        };
        robotInstance.transform.position = new Vector3(msg.start_x, msg.start_y, 0);
        robot.robotId = msg.robot_id;
        robot.endX = msg.end_x;
        robot.endY = msg.end_y;
        var path = new List<Vector3>();
        for (int i = 0; i < msg.path_x.Length; i++)
        {
            path.Add(new Vector3(msg.path_x[i], msg.path_y[i], 0));
        }
        robot.destinations = path;
        robot.loop = msg.loop;
        robot.moveSpeed = msg.move_speed;
        robot.perceptionRadius = msg.perception_radius;
        robot.obstacleDistanceThreshold = msg.obstacle_distance_threshold;
        robot.robotType = msg.robot_type;
        robotInstance.name = $"{msg.robot_type}_robot_{msg.robot_id}";
    }

    public static void SendTrackingData(RobotManagerTrackerMsg msg)
    {
        ros.Publish("robot_manager/subscribe_tracker", msg);
        //Debug.Log($"[RobotManagerClient] Published tracking data for Robot ID: {msg.robot_id}");
    }
}
