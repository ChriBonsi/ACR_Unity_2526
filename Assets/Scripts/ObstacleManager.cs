using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.ObstacleManager;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ObstacleManager
{
    private readonly string[] validTags = new string[]
    {
        "Robot",
        "UnattendedObstacle",
        "DirtObstacle",
    };
    private readonly Dictionary<int, GameObject> obstacles;
    private readonly ROSConnection ros;
    private readonly Robot robot;
    private readonly GameObject obstacleContainer;

    public ObstacleManager(Robot robot)
    {
        this.robot = robot;
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ObstacleManagerReportMsg>("obstacle_manager/report_obstacle", SubscribeCallback);
        obstacles = new();
        obstacleContainer = GameObject.Find("Obstacles");
    }

    public void ReportObstacle(GameObject obstacle, string status)
    {
        if (obstacle == null || string.IsNullOrEmpty(status) || !validTags.Contains(obstacle.tag) || obstacles.ContainsKey(obstacle.GetInstanceID())) return;
        PublishObstacle(obstacle, status);
        UpdateObstacles(obstacle, status);
    }

    public GameObject GetObstacle(int id)
    {
        GameObject obstacle = obstacles.GetValueOrDefault(id, null);
        return obstacle;
    }

    private void UpdateObstacles(GameObject obstacle, string status)
    {
        if (status == "handled")
        {
            if (obstacles.ContainsKey(obstacle.GetInstanceID()))
            {
                //Debug.Log($"[Robot {robotId}] Removing obstacle {obstacle.GetInstanceID()} from known obstacles.");
                obstacles.Remove(obstacle.GetInstanceID());
            }
            robot.OnObstacleHandled(obstacle.GetInstanceID());
        }
        else if (status == "unhandled")
        {
            if (!obstacles.ContainsKey(obstacle.GetInstanceID()))
            {
                //Debug.Log($"[Robot {robotId}] Adding obstacle {obstacle.GetInstanceID()} to known obstacles.");
                obstacles.Add(obstacle.GetInstanceID(), obstacle);
                robot.OnObstacleUnhandled(obstacle);
            }
        }
    }

    private void PublishObstacle(GameObject obstacle, string status)
    {
        //Debug.Log($"[Robot {robot.robotId}] Publishing obstacle {obstacle.GetInstanceID()} at {obstacle.transform.position} to all robots.");
        Transform transform = obstacle.transform;
        var msg = new ObstacleManagerReportMsg
        {
            robot_id = robot.robotId,
            id = obstacle.GetInstanceID().ToString(),
            x = transform.position.x,
            y = transform.position.y,
            z = transform.position.z,
            status = status,
            type = obstacle.tag,
            scale_x = transform.localScale.x,
            scale_y = transform.localScale.y,
            scale_z = transform.localScale.z,
        };
        ros.Publish("obstacle_manager/report_obstacle", msg);
    }

    private void SubscribeCallback(ObstacleManagerReportMsg msg)
    {
        //Debug.Log($"[Robot {robotId}] Received obstacle report for obstacle ID {msg.id} with status {msg.status}.");
        if (msg.robot_id == robot.robotId) return;
        GameObject gameObject = obstacleContainer.transform.Find(msg.id)?.gameObject;
        if (gameObject != null)
        {
            UpdateObstacles(gameObject, msg.status);
        }
        else
        {
            if (obstacles.ContainsKey(int.TryParse(msg.id, out int id) ? id : -1)) return;
            {
                obstacles.Remove(id);
                robot.OnObstacleHandled(id);
            }
        }
    }
}