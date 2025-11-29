using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.RobotManager;
using System.Collections.Generic;

public class RobotManagerClient : MonoBehaviour
{
    public GameObject robotPrefab;
    private static ROSConnection ros;
    public GameObject robotParent;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<RobotManagerRobotPublisherMsg>("robot_manager/publish_robot", SpawnRobots);

        var subscribeMsg = new RobotManagerRobotSubscriberMsg()
        {
            state = "ready"
        };

        /* var req = new ObstacleManagerSubscriberMsg()
        {
            x = 6,
            y = 6,
            type = "DirtObstacle",
        };
        ros.Publish("obstacle_manager/report_obstacle", req); */

        ros.Publish("robot_manager/request_robot", subscribeMsg);
    }

    void Update()
    {

    }

    private void SpawnRobots(RobotManagerRobotPublisherMsg msg)
    {
        Debug.Log("Spawning robot with ID: " + msg.robot_id);
        Debug.Log($"[RobotManagerClient] Spawning robot of type: {msg.robot_type} at ({msg.start_x}, {msg.start_y})");
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
    }

    public static void SendTrackingData(RobotManagerTrackerSubscriberMsg msg)
    {
        ros.Publish("robot_manager/subscribe_tracker", msg);
        //Debug.Log($"[RobotManagerClient] Published tracking data for Robot ID: {msg.robot_id}");
    }
}
