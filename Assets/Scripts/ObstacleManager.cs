using System.Collections.Generic;
using RosMessageTypes.ObstacleManager;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ObstacleManager
{
    private readonly Dictionary<int, GameObject> obstacles;
    private readonly ROSConnection ros;
    private readonly int robotId;
    private readonly GameObject obstacleContainer;

    public ObstacleManager(int robotId)
    {
        this.robotId = robotId;
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ObstacleManagerObstacleReportMsg>("obstacle_manager/report_obstacle", SubscribeCallback);
        obstacles = new();
        obstacleContainer = GameObject.Find("Obstacles");
    }

    public void ReportObstacle(GameObject obstacle, string status)
    {
        if(obstacle == null || string.IsNullOrEmpty(status)) return;
        PublishObstacle(obstacle, status);
        UpdateObstacles(obstacle, status);
    }

    private void UpdateObstacles(GameObject obstacle, string status)
    {
        if(status == "handled")
        {
            if(obstacles.ContainsKey(obstacle.GetInstanceID()))
            {
                Debug.Log($"[Robot {robotId}] Removing obstacle {obstacle.GetInstanceID()} from known obstacles.");
                obstacles.Remove(obstacle.GetInstanceID());
            }
        }
        else if(status == "unhandled")
        {
            if (!obstacles.ContainsKey(obstacle.GetInstanceID()))
            {
                Debug.Log($"[Robot {robotId}] Adding obstacle {obstacle.GetInstanceID()} to known obstacles.");
                obstacles.Add(obstacle.GetInstanceID(), obstacle);
            }
        }
    }

    private void PublishObstacle(GameObject obstacle, string status)
    {
        Debug.Log($"[Robot {robotId}] Publishing obstacle {obstacle.GetInstanceID()} to all robots.");
        Transform transform = obstacle.transform;
        var msg = new ObstacleManagerObstacleReportMsg
        {
            id = obstacle.GetInstanceID().ToString(),
            x = transform.position.x,
            y = transform.position.y,
            status = status,
            type = obstacle.tag,
            scale_x = transform.localScale.x,
            scale_y = transform.localScale.y,
        };
        ros.Publish("obstacle_manager/report_obstacle", msg);
    }

    private void SubscribeCallback(ObstacleManagerObstacleReportMsg msg)
    {
        Debug.Log($"[Robot {robotId}] Received obstacle report for obstacle ID {msg.id} with status {msg.status}.");
        GameObject gameObject = GameObject.Find(msg.id);
        if (gameObject != null)
        {
            UpdateObstacles(gameObject, msg.status);
        }
    }
}