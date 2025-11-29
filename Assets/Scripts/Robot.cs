using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PathPlanner;
using System.Collections.Generic;
using RosMessageTypes.ObstacleManager;
using RosMessageTypes.RobotManager;

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
    public float battery = 100f;

    [Header("Request")]
    public float endX = 3;
    public float endY = 3;

    protected ROSConnection ros;
    public Queue<Vector3> pathQueue = new();
    protected bool obstacleDetected = false;
    protected int destinationIndex = 1;
    protected bool isPathRequestPending = false;
    protected bool isPerformingTask = false;
    private float trackerTimer = 0f;
    private const float trackerInterval = 1f;
    private float startX;
    private float startY;
    private List<GameObject> reportedObstacles = new();
    protected GameObject icon;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PathPlannerResponseMsg>("path_planner/response", ResultCallback);

        SendRequest();

        icon = gameObject.transform.Find("TaskIcon").gameObject;
        icon.SetActive(false);

        perceptionRadius *= gameObject.transform.lossyScale.x;
    }

    void Update()
    {
        if (gameObject == null) return;

        //SendTrackingData();

        if (battery <= 0f)
        {
            return;
        }

        if(isPerformingTask)
        {
            UpdateTask();
            return;
        }
        
        if (pathQueue.Count == 0)
        {
            CheckAndAskForNewPath();
            return;
        }
        Vector3 target = pathQueue.Peek();
        CheckForObstacles(target);
        if (obstacleDetected)
        {
            SendRequest();
            return;
        }
        Move(target);
        CheckIfQueuedPointReached(target);
    }

    private void SendTrackingData()
    {
        trackerTimer += Time.deltaTime;
        if (trackerTimer >= trackerInterval)
        {
            var trackerMsg = new RobotManagerTrackerSubscriberMsg()
            {
                robot_id = robotId,
                current_x = gameObject.transform.position.x,
                current_y = gameObject.transform.position.y,
                start_x = startX,
                start_y = startY,
                end_x = endX,
                end_y = endY,
                robot_type = robotType,
                destinations_x = destinations.ConvertAll(v => v.x).ToArray(),
                destinations_y = destinations.ConvertAll(v => v.y).ToArray(),
                move_speed = moveSpeed,
                perception_radius = perceptionRadius,
                obstacle_distance_threshold = obstacleDistanceThreshold,
                loop = loop,
                obstacle_detected = obstacleDetected,
                performing_task = isPerformingTask
            };
            //Debug.Log($"[Robot {robotId}] Sent tracking data.");
            RobotManagerClient.SendTrackingData(trackerMsg);
            trackerTimer = 0f;
        }
    }

    private void CheckAndAskForNewPath()
    {
        if (loop && !isPathRequestPending)
        {
            destinationIndex = (destinationIndex + 1) % destinations.Count;
            Vector3 nextDestination = destinations[destinationIndex];
            endX = nextDestination.x;
            endY = nextDestination.y;
            SendRequest();
        }
    }

    protected void Move(Vector3 target)
    {
        gameObject.transform.position =
            Vector3.MoveTowards(gameObject.transform.position, target, moveSpeed * Time.deltaTime);
        UpdateBattery(-moveSpeed * Time.deltaTime * 0.1f);
    }

    protected void CheckIfQueuedPointReached(Vector3 target)
    {
        if (Vector3.Distance(gameObject.transform.position, target) < 0.02f)
        {
            pathQueue.Dequeue();
        }
    }

    private void CheckForObstacles(Vector3 target)
    {
        Vector3 currentPosition = gameObject.transform.position;
        RaycastHit2D[] hits = Physics2D.CircleCastAll(
            currentPosition,
            0.5f,
            (target - currentPosition).normalized,
            Vector3.Distance(currentPosition, target)
        );

        obstacleDetected = false;
        if (hits.Length == 0) return;

        //System.Array.Sort(hits, (x, y) => x.distance.CompareTo(y.distance));

        foreach (var hit in hits)
        {
            GameObject objectHit = hit.collider != null ? hit.collider.gameObject : null;
            if (objectHit == null || objectHit == gameObject) continue;
            //if (hit.distance > obstacleDistanceThreshold) continue;
            if (HandleSpecialObstacle(objectHit)) return;
            //if (obstacleDetected) return;
            ReportObstacle(objectHit, "unhandled");
            obstacleDetected = true;
            return;
        }
    }

    void OnCollisionEnter2D(Collision2D collision)
    {
        Debug.Log("Collision detected for Robot " + robotId);
        if (collision.gameObject.CompareTag("Robot"))
        {
            Debug.Log("Robot " + robotId + " collided with another robot.");
        }
    }

    protected void ReportObstacle(GameObject obstacle, string status)
    {
        if (reportedObstacles.Contains(obstacle)) return;
        var req = new ObstacleManagerObstacleReportMsg()
        {
            x = obstacle.transform.position.x,
            y = obstacle.transform.position.y,
            type = obstacle.tag,
            status = status,
            scale_x = obstacle.transform.localScale.x,
            scale_y = obstacle.transform.localScale.y,
            id = obstacle.GetInstanceID().ToString()
        };
        ros.Publish("obstacle_manager/report_obstacle", req);
        reportedObstacles.Add(obstacle);
        Debug.Log($"[Robot {robotId}] Reported obstacle {obstacle.GetInstanceID()} to Obstacle Manager.");
    }
    protected void UpdateBattery(float amount)
    {
        //Debug.Log($"[Robot {robotId}] Battery changed by {amount}.");
        battery = Mathf.Clamp(battery + amount, 0f, 100f);
    }

    protected virtual bool HandleSpecialObstacle(GameObject objectHit)
    {
        return false;
    }

    protected virtual void UpdateTask()
    {

    }

    public void SendRequest()
    {
        if (isPathRequestPending) return;
        Transform transform = gameObject.transform;
        float currentX = transform.position.x;
        float currentY = transform.position.y;
        var req = new PathPlannerRequestMsg()
        {
            robot_id = robotId,
            start_x = currentX,
            start_y = currentY,
            end_x = endX,
            end_y = endY
        };

        ros.Publish("path_planner/request", req);
        isPathRequestPending = true;
        Debug.Log($"[Robot {robotId}] Sent path request: ({currentX},{currentY}), ({endX},{endY})");
    }

    void ResultCallback(PathPlannerResponseMsg res)
    {
        if (res.robot_id != robotId) return;

        if (!res.success)
        {
            Debug.LogError($"[Robot {robotId}] Path planning failed.");
            isPathRequestPending = false;
            CheckAndAskForNewPath();
            return;
        }

        pathQueue.Clear();

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

        //obstacleDetected = false;
        isPathRequestPending = false;
        //Debug.Log($"[Robot {robotId}] Received path with {res.path_x.Length} points.");
    }
}
