using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PathPlanner;
using System.Collections.Generic;
using RosMessageTypes.RobotManager;
using System;

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
    public RobotState currentState = RobotState.Moving;
    public float endX = 3;
    public float endY = 3;
    public float endZ = 0;

    protected ROSConnection ros;
    public Queue<Vector3> pathQueue = new();
    protected int destinationIndex = 1;
    protected bool isPathRequestPending = false;
    private float startX;
    private float startY;
    private float startZ;
    protected GameObject icon;
    private GameObject currentRobotObstacle = null;
    public bool queueBackTaskState = false;
    private float trackerTimer = 0f;
    protected ObstacleManager obstacleManager;
    private bool chargeLock = false;
    protected readonly float chargeRate = 20f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PathPlannerResponseMsg>("path_planner/response", ResultCallback);

        obstacleManager = new(robotId);

        icon = transform.Find("TaskIcon").gameObject;
        icon.SetActive(false);

        perceptionRadius *= transform.lossyScale.x;

        destinationIndex = destinations.FindIndex(
            v => v.x == endX && v.y == endY && v.z == endZ
        );

        SendRequest();
    }

    void Update()
    {
        if (gameObject == null) return;

        if(RobotState.Deadlock == currentState) return;

        if (battery <= 0f)
        {
            // Request battery change
            return;
        }

        if(battery < 10f && !chargeLock)
        {
            Debug.LogWarning($"[Robot {robotId}] Battery low: {battery}%, requesting recharge.");
            SendBatteryRechargeRequest();
            return;
        }

        //SendTrackingData();

        if(currentState == RobotState.Moving)
        {
            CheckSensors();
        }

        switch (currentState)
        {
            case RobotState.Moving:
            Move();
            if(robotType == "baggage"){
                queueBackTaskState = false;
                if(Vector3.Distance(transform.position, new Vector3(endX, endY, endZ)) < 0.1f){
                    currentState = RobotState.PerformingTask;
                    pathQueue.Clear();
                    break;
                }
            }
            if(CheckIfChargingStationReached()) break;
            CheckIfQueuedPointReached();
            if(pathQueue.Count == 0) CheckAndAskForNewPath();
            break;
            case RobotState.Yielding:
            CheckIfYieldIsDone();
            break;
            case RobotState.PerformingTask:
            UpdateTask();
            break;
            case RobotState.WaitingForPath:
            break;
            case RobotState.Charging:
            ChargeRobot();
            break;
        }
    }

    protected virtual int GetPriority() { return 0; }

    private void SendTrackingData()
    {
        trackerTimer += Time.deltaTime;
        if (trackerTimer >= 1f)
        {
            var trackerMsg = new RobotManagerTrackerMsg()
            {
                robot_id = robotId,
                current_x = transform.position.x,
                current_y = transform.position.y,
                current_z = transform.position.z,
                start_x = startX,
                start_y = startY,
                start_z = startZ,
                end_x = endX,
                end_y = endY,
                end_z = endZ,
                robot_type = robotType,
                destinations_x = destinations.ConvertAll(v => v.x).ToArray(),
                destinations_y = destinations.ConvertAll(v => v.y).ToArray(),
                destinations_z = destinations.ConvertAll(v => v.z).ToArray(),
                move_speed = moveSpeed,
                perception_radius = perceptionRadius,
                obstacle_distance_threshold = obstacleDistanceThreshold,
                loop = loop,
                obstacle_detected = false,
                performing_task = false
            };
            //Debug.Log($"[Robot {robotId}] Sent tracking data.");
            RobotManagerClient.SendTrackingData(trackerMsg);
            trackerTimer = 0f;
        }
    }

    private void ChargeRobot()
    {
        UpdateBattery(chargeRate * Time.deltaTime);
        if(battery >= 100f)
        {
            Debug.Log($"[Robot {robotId}] Fully charged. Resuming tasks.");
            SetNextClosestDestination();
            SendRequest();
            chargeLock = false;
        }
    }

    protected void CheckAndAskForNewPath()
    {
        if (destinations.Count > 0 && !isPathRequestPending)
        {
            if (loop)
            {
                destinationIndex = (destinationIndex + 1) % destinations.Count;
            }
            else
            {
                if (destinationIndex >= destinations.Count - 1) return;
                destinationIndex++;
            }
            Vector3 nextDestination = destinations[destinationIndex];
            endX = nextDestination.x;
            endY = nextDestination.y;
            endZ = nextDestination.z;
            SendRequest();
        }
    }

    protected void Move()
    {
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        transform.position =
            Vector3.MoveTowards(transform.position, target, moveSpeed * Time.deltaTime);
        UpdateBattery(-moveSpeed * Time.deltaTime * 0.1f);
    }

    protected bool CheckIfChargingStationReached()
    {
        if(!chargeLock) return false;
        if(pathQueue.Count == 0) return false;
        Vector3 lastPoint = pathQueue.ToArray()[pathQueue.Count - 1];
        if(Vector3.Distance(transform.position, lastPoint) < 0.1f)
        {
            currentState = RobotState.Charging;
            Debug.Log($"[Robot {robotId}] Reached charging station. Starting to charge.");
            return true;
        }
        return false;
    }

    protected void CheckIfQueuedPointReached()
    {
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        if (Vector3.Distance(transform.position, target) < 0.02f)
        {
            pathQueue.Dequeue();
        }
    }

    protected void CheckSensors()
    {
        if(pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        Vector3 currentPosition = transform.position;
        Vector3 direction = target - currentPosition;
        if(direction == Vector3.zero) return;
        
        RaycastHit[] hits = Physics.SphereCastAll(
            currentPosition,
            0.5f,
            direction.normalized,
            2f
            //Vector3.Distance(currentPosition, target)
        );

        if (hits.Length == 0) return;

        Array.Sort(hits, (x, y) => x.distance.CompareTo(y.distance));

        foreach (var hit in hits)
        {
            //Debug.LogWarning($"[Robot {robotId}] ${hit.collider.isTrigger}");
            if(hit.collider == null || !hit.collider.isTrigger) continue;
            GameObject objectHit = hit.collider.gameObject;
            if (objectHit == null || objectHit == gameObject) continue;

            // Dynamic robot-robot
            if(objectHit.CompareTag("Robot"))
            {
                Robot otherRobot = objectHit.GetComponent<Robot>();
                HandleRobotInteraction(otherRobot, hit.distance);
                return;
            }

            if(hit.distance <= obstacleDistanceThreshold)
            {
                HandleStaticObstacle(objectHit, hit.distance);
                return;
            }
        }
    }

    private void HandleRobotInteraction(Robot otherRobot, float distance)
    {
        if(distance > obstacleDistanceThreshold + 0.5f) return;

        int myPriority = GetPriority();
        int otherPriority = otherRobot.GetPriority();
        bool yield = false;

        if (myPriority < otherPriority) 
        {
            yield = true;
        }
        else if (myPriority == otherPriority)
        {
            if (robotId < otherRobot.robotId) yield = true;
        }

        if (yield)
        {
            Debug.Log($"[Robot {robotId}] Yielding to Robot {otherRobot.robotId}.");
            currentRobotObstacle = otherRobot.gameObject;
            currentState = RobotState.Yielding;
        }
    }

    private void HandleStaticObstacle(GameObject objectHit, float distance)
    {
        if(HandleSpecialObstacle(objectHit)) return;
        Vector3 target = pathQueue.Peek();
        float distanceToTarget = Vector3.Distance(transform.position, target);
        if(distanceToTarget <= distance) return;
        obstacleManager.ReportObstacle(objectHit, "unhandled");
        SendRequest();
    }

    private void CheckIfYieldIsDone()
    {
        Collider[] hits = Physics.OverlapSphere(transform.position, perceptionRadius);
        bool isClear = true;

        foreach (var hit in hits)
        {
            if (hit.gameObject == gameObject) continue;
            if (hit.CompareTag("Robot"))
            {
                isClear = false;
                break;
            }
        }

        if (isClear)
        {
            currentState = !queueBackTaskState ? RobotState.Moving : RobotState.PerformingTask;
            currentRobotObstacle = null;
            Debug.Log($"[Robot {robotId}] Yield clear. Resuming movement.");
        }
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

    protected void SetNextClosestDestination()
    {
        int idx = -1;
        float minDistance = float.PositiveInfinity;
        for (int i = 0; i < destinations.Count; i++)
        {
            float distance = Vector3.Distance(transform.position, destinations[i]);
            if (distance < minDistance)
            {
                minDistance = distance;
                idx = i;        
            }
        }
        destinationIndex = (idx - 1) % destinations.Count;
    }

    protected void SendRequest()
    {
        if (isPathRequestPending) return;
        //if(currentState == RobotState.HandlingObstacle) queueBackTaskState = true;
        currentState = RobotState.WaitingForPath;

        float currentX = transform.position.x;
        float currentY = transform.position.y;
        float currentZ = transform.position.z;
        var req = new PathPlannerRequestMsg()
        {
            robot_id = robotId,
            start_x = currentX,
            start_y = currentY,
            start_z = currentZ,
            end_x = endX,
            end_y = endY,
            end_z = endZ
        };

        ros.Publish("path_planner/request", req);
        isPathRequestPending = true;
        Debug.Log($"[Robot {robotId}] Sent path request: ({currentX},{currentY},{currentZ}), ({endX},{endY},{endZ})");
    }

    private void SendBatteryRechargeRequest()
    {
        if (isPathRequestPending) return;
        currentState = RobotState.WaitingForPath;

        float currentX = transform.position.x;
        float currentY = transform.position.y;
        float currentZ = transform.position.z;

        var req = new PathPlannerBatteryRequestMsg()
        {
            robot_id = robotId,
            start_x = currentX,
            start_y = currentY,
            start_z = currentZ,
        };

        ros.Publish("path_planner/battery_request", req);
        isPathRequestPending = true;
        chargeLock = true;
        Debug.Log($"[Robot {robotId}] Sent battery recharge path request.");
    }

    private void ResultCallback(PathPlannerResponseMsg res)
    {
        if (res.robot_id != robotId) return;

        if (!res.success)
        {
            Debug.LogError($"[Robot {robotId}] Path planning failed.");
            currentState = RobotState.Deadlock;
            isPathRequestPending = false;
            return;
        }

        pathQueue.Clear();

        for (int i = 0; i < res.path_x.Length; i++)
        {
            Vector3 point = new(
                res.path_x[i],
                res.path_y[i],
                res.path_z[i]
            );
            pathQueue.Enqueue(point);
        }

        isPathRequestPending = false;
        currentState = !queueBackTaskState ? RobotState.Moving : RobotState.PerformingTask;
        //Debug.Log($"[Robot {robotId}] Received path with {res.path_x.Length} points.");
    }
}
