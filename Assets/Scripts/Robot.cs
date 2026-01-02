using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PathPlanner;
using RosMessageTypes.Std;
using System.Collections.Generic;
using RosMessageTypes.RobotManager;
using System;

public class Robot : MonoBehaviour
{
    [Serializable]
    private class RobotCoordination
    {
        public int target_robot_id;
        public string command;
        public float x;
        public float y;
        public float z;
    }

    [Header("Robot settings")]
    public int robotId = 1;
    public float moveSpeed = 2f;
    public float perceptionRadius = 0.5f;
    public float obstacleDistanceThreshold = 2f;
    public string robotType = "default";
    public List<Vector3> destinations = new();
    public bool loop = false;
    public RobotState currentState = RobotState.Moving;
    public float endX;
    public float endY;
    public float endZ;

    protected ROSConnection ros;
    public Queue<Vector3> pathQueue = new();
    protected int destinationIndex = 1;
    protected bool isPathRequestPending = false;
    private float startX;
    private float startY;
    private float startZ;
    protected GameObject icon;
    public bool queueBackTaskState = false;
    private float trackerTimer = 0f;
    protected ObstacleManager obstacleManager;
    protected Battery battery;

    private Vector3 yieldTargetPosition;
    private Vector3 yieldReturnPosition;
    private bool isMovingToYield = false;
    private bool isReturningFromYield = false;
    private bool isPausedForSafety = false;
    private readonly Dictionary<int, float> lastCommandTime = new();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PathPlannerResponseMsg>("path_planner/response", ResultCallback);
        ros.Subscribe<StringMsg>("robot_coordination", CoordinationCallback);

        obstacleManager = new(robotId);
        battery = new(robotId);

        icon = transform.Find("TaskIcon").gameObject;
        icon.SetActive(false);

        destinationIndex = destinations.FindIndex(
            v => v.x == endX && v.y == endY && v.z == endZ
        );

        SendRequest();
    }

    void Update()
    {
        if (gameObject == null) return;

        if (currentState == RobotState.Deadlock) return;

        float currentBattery = battery.GetBattery();

        if (currentBattery <= 0f)
        {
            // Request battery change
            return;
        }

        if (currentBattery <= 10f && !battery.IsChargeLocked())
        {
            Debug.LogWarning($"[Robot {robotId}] Battery low: {currentBattery}%, requesting recharge.");
            SendBatteryRechargeRequest();
            return;
        }

        if (currentState == RobotState.Moving)
        {
            CheckSensors();
        }

        switch (currentState)
        {
            case RobotState.Moving:
                Move();
                if (CheckBaggageReachedDestination()) break;
                if (CheckIfChargingStationReached()) break;
                CheckIfQueuedPointReached();
                if (pathQueue.Count == 0) CheckAndAskForNewPath();
                break;
            case RobotState.Yielding:
                YieldBehavior();
                break;
            case RobotState.PerformingTask:
                UpdateTask();
                break;
            case RobotState.WaitingForPath:
                break;
            case RobotState.Charging:
                ChargeAndCheck();
                break;
        }
    }

    private bool CheckBaggageReachedDestination()
    {
        if (robotType != "baggage") return false;

        if (Vector3.Distance(transform.position, new Vector3(endX, endY, endZ)) < 0.1f)
        {
            currentState = RobotState.PerformingTask;
            pathQueue.Clear();
            return true;
        }
        return false;
    }

    private void ChargeAndCheck()
    {
        battery.ChargeRobot();
        if (battery.GetBattery() >= 100f)
        {
            Debug.Log($"[Robot {robotId}] Fully charged. Resuming tasks.");
            SetNextClosestDestination();
            SendRequest();
            SetRobotVisibility(true);
            battery.SetChargeLock(false);
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
        if (isPausedForSafety) return;
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        transform.position =
            Vector3.MoveTowards(transform.position, target, moveSpeed * Time.deltaTime);
        battery.UpdateBattery(-moveSpeed * Time.deltaTime * 0.1f);
    }

    protected bool CheckIfChargingStationReached()
    {
        if (!battery.IsChargeLocked()) return false;
        if (pathQueue.Count == 0) return false;
        Vector3 lastPoint = pathQueue.ToArray()[pathQueue.Count - 1];
        if (Vector3.Distance(transform.position, lastPoint) < 0.1f)
        {
            currentState = RobotState.Charging;
            SetRobotVisibility(false);
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
        isPausedForSafety = false;
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        Vector3 currentPosition = transform.position;
        Vector3 direction = target - currentPosition;
        if (direction == Vector3.zero) return;

        Collider[] colliderHits = Physics.OverlapSphere(currentPosition, obstacleDistanceThreshold);

        Array.Sort(colliderHits, (x, y) =>
            Vector3.Distance(currentPosition, x.transform.position)
            .CompareTo(Vector3.Distance(currentPosition, y.transform.position))
        );

        foreach (var collider in colliderHits)
        {
            if (collider == null || !collider.isTrigger) continue;
            GameObject objectHit = collider.gameObject;
            if (objectHit == null || objectHit == gameObject) continue;
            float distance = Vector3.Distance(currentPosition, objectHit.transform.position);

            // Dynamic robot-robot
            if (objectHit.CompareTag("Robot"))
            {
                Robot otherRobot = objectHit.GetComponent<Robot>();
                HandleRobotInteraction(otherRobot, distance);
                return;
            }

            // Static obstacles
            // Should always be true (distance inside radius) unless distance threshold used in OverlapSphere is different
            if (distance <= obstacleDistanceThreshold)
            {
                HandleStaticObstacle(objectHit, distance);
                return;
            }
        }
    }

    private bool CheckPrecedence(Robot otherRobot)
    {
        int myPriority = GetPriority();
        int otherPriority = otherRobot.GetPriority();

        if (myPriority > otherPriority)
        {
            return true;
        }
        else if (myPriority == otherPriority)
        {
            if (robotId > otherRobot.robotId) return true;
        }

        return false;
    }

    private void PauseForSafety(Robot otherRobot, float distance)
    {
        float myRadius = transform.lossyScale.x / 2f;
        float otherRadius = otherRobot.transform.lossyScale.x / 2f;
        float safeDistance = myRadius + otherRadius + 0.2f;
        //Debug.Log($"[Robot {robotId}] Higher priority than Robot {otherRobot.robotId}. Maintaining safe distance {safeDistance}.");

        if (distance < safeDistance)
        {
            isPausedForSafety = true;
        }
    }

    private void SendYieldCommand(Robot otherRobot, Vector3 yieldPos)
    {
        RobotCoordination data = new()
        {
            target_robot_id = otherRobot.robotId,
            command = "yield",
            x = yieldPos.x,
            y = yieldPos.y,
            z = yieldPos.z
        };

        StringMsg msg = new(JsonUtility.ToJson(data));
        Debug.Log($"[Robot {robotId}] Commanded Robot {otherRobot.robotId} to yield at {yieldPos}");
        ros.Publish("robot_coordination", msg);

    }

    private void HandleRobotInteraction(Robot otherRobot, float distance)
    {
        // Should always be false (distance inside radius) unless distance threshold used in OverlapSphere is different
        if (distance > obstacleDistanceThreshold) return;

        bool precedence = CheckPrecedence(otherRobot);

        if (!precedence)
        {
            if (currentState != RobotState.Yielding)
            {
                Debug.Log($"[Robot {robotId}] Lower priority than Robot {otherRobot.robotId}. Waiting for instructions.");
                currentState = RobotState.Yielding;
                isMovingToYield = false;
                isReturningFromYield = false;
                //obstacleManager.ReportObstacle(gameObject, "unhandled");
            }
        }
        else
        {
            PauseForSafety(otherRobot, distance);

            if (!lastCommandTime.ContainsKey(otherRobot.robotId) || Time.time - lastCommandTime[otherRobot.robotId] > 3f)
            {
                if (FindYieldPosition(otherRobot, out Vector3 yieldPos))
                {
                    SendYieldCommand(otherRobot, yieldPos);
                    lastCommandTime[otherRobot.robotId] = Time.time;
                }
                else
                {
                    Debug.LogWarning($"[Robot {robotId}] Unable to find yield position for Robot {otherRobot.robotId}. Default: {yieldPos}");
                }
            }
        }
    }

    private void HandleStaticObstacle(GameObject objectHit, float distance)
    {
        if (HandleSpecialObstacle(objectHit)) return;
        Vector3 target = pathQueue.Peek();
        float distanceToTarget = Vector3.Distance(transform.position, target);
        //Debug.Log($"[Robot {robotId}] Detected static obstacle: {objectHit.name} at distance {distance}. Distance to target: {distanceToTarget}");
        //UnityEditor.EditorApplication.isPaused = true;

        Vector3 direction = (target - transform.position).normalized;
        if (Physics.SphereCast(transform.position, perceptionRadius, direction, out RaycastHit hit, distanceToTarget))
        {
            if (hit.collider.gameObject == objectHit)
            {
                obstacleManager.ReportObstacle(objectHit, "unhandled");
                SendRequest();
            }
        }
    }

    private void YieldBehavior()
    {
        if (isMovingToYield)
        {
            transform.position = Vector3.MoveTowards(transform.position, yieldTargetPosition, moveSpeed * Time.deltaTime);
            if (Vector3.Distance(transform.position, yieldTargetPosition) < 0.02f)
            {
                isMovingToYield = false;
            }
            return;
        }

        if (isReturningFromYield)
        {
            transform.position = Vector3.MoveTowards(transform.position, yieldReturnPosition, moveSpeed * Time.deltaTime);
            if (Vector3.Distance(transform.position, yieldReturnPosition) < 0.02f)
            {
                isReturningFromYield = false;
                currentState = !queueBackTaskState ? RobotState.Moving : RobotState.PerformingTask;
                //obstacleManager.ReportObstacle(gameObject, "handled");
                Debug.Log($"[Robot {robotId}] Yield complete. Resuming.");
            }
            return;
        }

        if (FindBestReturnPoint(out Vector3 returnPos))
        {
            if (IsSafeToReturn(returnPos))
            {
                yieldReturnPosition = returnPos;
                UpdatePathQueue(returnPos);
                isReturningFromYield = true;
            }
        }
    }

    private bool IsSafeToReturn(Vector3 targetPos)
    {
        Collider[] hits = Physics.OverlapSphere(targetPos, obstacleDistanceThreshold);
        foreach (var hit in hits)
        {
            if (hit.gameObject == gameObject) continue;
            if (hit.CompareTag("Robot")) return false;
        }
        return true;
    }

    private bool FindBestReturnPoint(out Vector3 returnPos)
    {
        returnPos = Vector3.zero;
        if (pathQueue.Count == 0) return false;

        Vector3[] pathPoints = pathQueue.ToArray();
        int closestIndex = -1;
        float minDistance = float.MaxValue;

        for (int i = 0; i < pathPoints.Length; i++)
        {
            float d = Vector3.Distance(transform.position, pathPoints[i]);
            if (d < minDistance)
            {
                minDistance = d;
                closestIndex = i;
            }
        }

        if (closestIndex != -1)
        {
            returnPos = pathPoints[closestIndex];
            return true;
        }
        return false;
    }

    private void UpdatePathQueue(Vector3 startPoint)
    {
        Vector3[] pathPoints = pathQueue.ToArray();
        pathQueue.Clear();
        bool found = false;
        foreach (var point in pathPoints)
        {
            if (!found && Vector3.Distance(point, startPoint) < 0.01f)
            {
                found = true;
            }
            if (found)
            {
                pathQueue.Enqueue(point);
            }
        }
    }

    private void CoordinationCallback(StringMsg msg)
    {
        RobotCoordination data = JsonUtility.FromJson<RobotCoordination>(msg.data);
        if (data.target_robot_id != robotId) return;
        if (data.command == "yield")
        {
            Vector3 targetPos = new(data.x, data.y, data.z);
            if (yieldTargetPosition == targetPos) return;
            Debug.Log($"[Robot {robotId}] Received yield command to {targetPos}");
            currentState = RobotState.Yielding;
            yieldTargetPosition = targetPos;
            yieldReturnPosition = transform.position;
            isMovingToYield = true;
            isReturningFromYield = false;
            //obstacleManager.ReportObstacle(gameObject, "unhandled");
        }
    }

    private bool FindYieldPosition(Robot otherRobot, out Vector3 yieldPos)
    {
        Vector3 otherRobotPos = otherRobot.transform.position;
        yieldPos = otherRobotPos;
        Vector3 myPos = transform.position;

        float[] checkDistances = new float[] { 1f };
        float angleCheck = 45f;
        int checkDirections = Mathf.CeilToInt(360f / angleCheck);

        foreach (float dist in checkDistances)
        {
            for (int i = 0; i < checkDirections; i++)
            {
                float angle = i * angleCheck;
                Quaternion rotation = Quaternion.Euler(0, angle, 0);
                Vector3 dir = rotation * Vector3.forward;

                Vector3 candidatePos = otherRobotPos + dir * dist;

                if (IsPositionValid(candidatePos, otherRobot) && IsSafeFromPath(candidatePos, this))
                {
                    if (Vector3.Distance(candidatePos, myPos) < Vector3.Distance(otherRobotPos, myPos))
                        continue;

                    Debug.Log($"[Robot {robotId}] Found yield position at {candidatePos} with distance {dist} and angle {angle}");
                    //yieldPos = candidatePos;
                    yieldPos = new(Mathf.Round(candidatePos.x * 100f) / 100f,
                                         Mathf.Round(candidatePos.y * 100f) / 100f,
                                         Mathf.Round(candidatePos.z * 100f) / 100f);
                    return true;
                }
            }
        }
        return false;
    }

    private bool IsSafeFromPath(Vector3 target, Robot robotToAvoid)
    {
        if (robotToAvoid.pathQueue == null || robotToAvoid.pathQueue.Count == 0) return true;

        foreach (var point in robotToAvoid.pathQueue)
        {
            if (Vector3.Distance(target, point) < 1f) return false;
        }
        return true;
    }

    private bool IsPositionValid(Vector3 pos, Robot otherRobot)
    {
        Collider[] hits = Physics.OverlapSphere(pos, 0.45f);
        foreach (var hit in hits)
        {
            if (hit.gameObject == gameObject) continue;
            if (hit.CompareTag("Invalid")) return false;
            if (hit.CompareTag("Robot")) return false;
            //if (hit.GetComponent<Robot>() != null) return false;
        }
        return true;
    }

    protected virtual bool HandleSpecialObstacle(GameObject objectHit)
    {
        return false;
    }

    protected virtual void UpdateTask()
    {
    }

    protected void SetRobotVisibility(bool visible)
    {
        foreach (var r in GetComponentsInChildren<Renderer>()) r.enabled = visible;
        foreach (var c in GetComponentsInChildren<Collider>()) c.enabled = visible;
        foreach (var c in GetComponentsInChildren<Canvas>()) c.enabled = visible;
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
        if (idx != -1)
        {
            destinationIndex = idx % destinations.Count;
            endX = destinations[destinationIndex].x;
            endY = destinations[destinationIndex].y;
            endZ = destinations[destinationIndex].z;
        }
    }

    protected void SendRequest()
    {
        if (isPathRequestPending) return;
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
        //Debug.Log($"[Robot {robotId}] Sent path request: ({currentX},{currentY},{currentZ}), ({endX},{endY},{endZ})");
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
        battery.SetChargeLock(true);
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
            battery.SetChargeLock(false);
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
