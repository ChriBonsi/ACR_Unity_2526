using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PathPlanner;
using System.Collections.Generic;
using RosMessageTypes.ObstacleManager;
using System.Collections;

public class Robot : MonoBehaviour
{
    [Header("Robot settings")]
    public int robotId = 1;
    public float moveSpeed = 3f;
    public float perceptionRadius = 1f;
    public float obstacleDistanceThreshold = 1f;
    public string robotType = "default";
    public List<Vector3> destinations = new();
    public bool loop = false;

    [Header("Request")]
    public float endX = 3;
    public float endY = 3;

    protected ROSConnection ros;
    protected Queue<Vector3> pathQueue = new();
    protected bool obstacleDetected = false;
    private int destinationIndex = 0;
    private bool isPathRequestPending = false;
    protected bool isPerformingTask = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PathPlannerFeedbackMsg>("path_planner/feedback", FeedbackCallback);
        ros.Subscribe<PathPlannerResponseMsg>("path_planner/response", ResultCallback);

        if (destinations.Count == 0) SendRequest();

        perceptionRadius *= gameObject.transform.lossyScale.x;
    }

    void Update()
    {
        if (gameObject == null) return;

        if (pathQueue.Count == 0)
        {
            CheckAndAskForNewPath();
            return;
        }

        if (isPerformingTask) return;

        Vector3 target = pathQueue.Peek();
        CheckForObstacles(target);
        if (obstacleDetected) return;
        Move(target);
        if (Vector3.Distance(gameObject.transform.position, target) < 0.02f)
        {
            pathQueue.Dequeue();
        }
    }

    private void CheckAndAskForNewPath()
    {
        if (loop && !isPathRequestPending)
        {
            destinationIndex = (destinationIndex + 1) % destinations.Count;
            endX = destinations[destinationIndex].x;
            endY = destinations[destinationIndex].y;
            SendRequest();
        }
    }

    private void Move(Vector3 target)
    {
        gameObject.transform.position =
            Vector3.MoveTowards(gameObject.transform.position, target, moveSpeed * Time.deltaTime);
    }

    private void CheckForObstacles(Vector3 target)
    {
        RaycastHit2D[] hits = Physics2D.CircleCastAll(gameObject.transform.position, perceptionRadius, target - gameObject.transform.position,
            Vector3.Distance(gameObject.transform.position, target));
        
        if(hits.Length == 0)
        {
            obstacleDetected = false;
            return;
        }

        foreach (var hit in hits)
        {
            GameObject objectHit = hit.collider != null ? hit.collider.gameObject : null;
            if (objectHit != null && objectHit != gameObject)
            {
                if (hit.distance < obstacleDistanceThreshold)
                {
                    if (HandleSpecialObstacle(objectHit)) 
                    {
                        continue;
                    }

                    if (!obstacleDetected)
                    {
                        var req = new ObstacleManagerSubscriberMsg()
                        {
                            x = hit.transform.position.x,
                            y = hit.transform.position.y,
                            type = objectHit.tag
                        };
                        ros.Publish("obstacle_manager/report_obstacle", req);
                        Debug.Log($"[Robot {robotId}] Reported obstacle at ({req.x}, {req.y}) to Obstacle Manager.");
                        obstacleDetected = true;
                        return;
                    }
                }
            }
        }
    }

    protected virtual bool HandleSpecialObstacle(GameObject objectHit)
    {
        return false;
    }

    public void SendRequest()
    {
        Transform transform = gameObject.transform;
        float startX = transform.position.x;
        float startY = transform.position.y;
        var req = new PathPlannerRequestMsg()
        {
            robot_id = robotId,
            start_x = startX,
            start_y = startY,
            end_x = endX,
            end_y = endY
        };

        ros.Publish("path_planner/request", req);
        isPathRequestPending = true;
        Debug.Log($"[Robot {robotId}] Sent path request: ({startX},{startY}), ({endX},{endY})");
    }

    void FeedbackCallback(PathPlannerFeedbackMsg fb)
    {
        if (fb.robot_id != robotId) return;

        Debug.Log($"[Robot {robotId}] {fb.percent_complete:F1}%");
    }

    void ResultCallback(PathPlannerResponseMsg res)
    {
        if (res.robot_id != robotId) return;

        if (!res.success)
        {
            Debug.LogWarning($"[Robot {robotId}] Path planning failed.");
            return;
        }

        //pathQueue.Clear();

        for (int i = 0; i < res.path_x.Length; i++)
        {
            Vector3 point = new(
                res.path_x[i],
                res.path_y[i],
                0f
            );
            //if (Vector3.Distance(robot.transform.position, point) < 0.05f) continue;

            pathQueue.Enqueue(point);
        }

        isPathRequestPending = false;
        Debug.Log($"[Robot {robotId}] Received path with {res.path_x.Length} points.");
    }
}
