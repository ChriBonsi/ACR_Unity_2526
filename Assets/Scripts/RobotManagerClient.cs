using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.RobotManager;
using System.Collections.Generic;

public class RobotManagerClient : MonoBehaviour
{
    public GameObject robotPrefab;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<RobotManagerPublisherMsg>("robot_manager/publish_robot", SpawnRobots);

        var subscribeMsg = new RobotManagerSubscriberMsg()
        {
            state = "ready"
        };

        ros.Publish("robot_manager/subscribe_robot", subscribeMsg);
    }

    void Update()
    {

    }

    private void SpawnRobots(RobotManagerPublisherMsg msg)
    {
        Debug.Log("Spawning robot with ID: " + msg.robot_id);
        GameObject robotInstance = Instantiate(robotPrefab);
        Robot robot = msg.robot_type switch
        {
            "cleaner" => robotInstance.AddComponent<CleanerRobot>(),
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
}
