using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PathPlanner;
using RosMessageTypes.Std;
using System.Collections.Generic;
using RosMessageTypes.RobotManager;
using System;
using System.Linq;

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
    private float trackerTimer = 0f;
    protected ObstacleManager obstacleManager;
    protected Battery battery;
    private float deadLockTimer = 0f;
    private float yieldTimer = 0f;
    private float isPausedTimer = 0f;
    private Vector3 yieldTargetPosition;
    //private Vector3 yieldReturnPosition;
    private bool isMovingToYield = false;
    //private bool isReturningFromYield = false;
    public bool isPausedForSafety = false;
    private readonly Dictionary<int, float> lastCommandTime = new();
    protected string lastNodeKey = "";
    private Vector3 lastPosition = Vector3.zero;
    private float safeDistanceThreshold = 1f;
    private readonly RobotState[] priorityStates = new RobotState[]
    {
        RobotState.Deadlock,
        RobotState.Yielding,
        RobotState.WaitingForPath
    };

    protected void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PathPlannerResponseMsg>("path_planner/response", PathResultCallback);
        ros.Subscribe<StringMsg>("robot_coordination", RobotCoordinationCallback);

        obstacleManager = new(this);
        battery = new(robotId);

        icon = transform.Find("TaskIcon").gameObject;
        icon.SetActive(false);

        destinationIndex = destinations.FindIndex(
            v => v.x == endX && v.y == endY && v.z == endZ
        );

        SendPathRequest();
    }

    protected void Update()
    {
        if (gameObject == null) return;

        float currentBattery = battery.GetBattery();

        if (currentBattery <= 0f)
        {
            Debug.LogError($"[Robot {robotId}] Battery depleted! Robot shutting down.");
            // Request battery change
            return;
        }

        if (currentState == RobotState.Deadlock || currentState == RobotState.WaitingForPath)
        {
            deadLockTimer += Time.deltaTime;
            if (deadLockTimer >= 5f)
            {
                Debug.LogWarning($"[Robot {robotId}] Deadlock timeout reached. Attempting to move again.");
                deadLockTimer = 0f;
                //currentState = RobotState.Moving;
            }
            return;
        }

        if (yieldTimer >= 5f)
        {
            Debug.LogWarning($"[Robot {robotId}] Yield timeout reached. Resuming movement.");
            yieldTimer = 0f;
            //isReturningFromYield = false;
            //isMovingToYield = false;
            //currentState = RobotState.Moving;
        }

        if (currentState == RobotState.Moving && isPausedForSafety)
        {
            isPausedTimer += Time.deltaTime;
            lastPosition = transform.position;
            if (isPausedTimer >= 5f)
            {
                Debug.LogWarning($"[Robot {robotId}] Movement paused. Attempting to move again.");
                isPausedTimer = 0f;
                //SendRequest();
            }
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
                if (CheckDestinationReached()) break;
                if (CheckIfChargingStationReached()) break;
                CheckIfQueuedPointReached();
                Vector3 nextDestination = GetNextDestination();
                SetGoal(nextDestination);
                SendPathRequest();
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

    private void CheckSensors()
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

    private void HandleRobotInteraction(Robot otherRobot, float distance)
    {
        // Should always be false (distance inside radius) unless distance threshold used in OverlapSphere is different
        if (distance > obstacleDistanceThreshold) return;
        if (!IsBlockingMyPath(otherRobot)) return;

        bool precedence = CheckPrecedence(otherRobot);

        if (!precedence)
        {
            // If actively moving to a yield position, be less sensitive to blocking as it is trying to clear the way. However, strictly enforce physical safety.
            if (isMovingToYield)
            {
                /* float myRadius = transform.lossyScale.x / 2f;
                float otherRadius = otherRobot.transform.lossyScale.x / 2f;
                float safeDistance = myRadius + otherRadius + 0.1f;
                if (distance < safeDistance)
                {
                    isPausedForSafety = true;
                } */
                PauseForSafety(otherRobot, distance);
            }
            // Pause if the other robot blocks our path
            else if (!isPausedForSafety)
            {
                //Debug.Log($"[Robot {robotId}] Lower priority than Robot {otherRobot.robotId}. Waiting for instructions.");
                //currentState = RobotState.Yielding;
                isPausedForSafety = true;
                isMovingToYield = false;
                //isReturningFromYield = false;
                //obstacleManager.ReportObstacle(gameObject, "unhandled");
            }
        }
        else
        {
            PauseForSafety(otherRobot, distance);
            bool timeExpired = !lastCommandTime.ContainsKey(otherRobot.robotId) || Time.time - lastCommandTime[otherRobot.robotId] > 1f;
            if (timeExpired)
            {
                if (FindYieldPosition(otherRobot, out Vector3 yieldPos))
                {
                    SendYieldCommand(otherRobot, yieldPos);
                    lastCommandTime[otherRobot.robotId] = Time.time;
                }
                else
                {
                    Debug.Log($"[Robot {robotId}] No valid yield position found for Robot {otherRobot.robotId}. Maintaining position.");
                }
            }
        }
    }

    /// <summary>
    /// Checks if this robot has precedence over another robot based on priority, state, and ID.
    /// This robot has precedence if it has a higher priority value, or if the other robot's state is in the state list, or if this robot's id is higher
    /// </summary>
    /// <param name="otherRobot">The other robot to compare against.</param>
    /// <returns>True if this robot has precedence, false otherwise.</returns>
    private bool CheckPrecedence(Robot otherRobot)
    {
        int myPriority = GetPriority();
        int otherPriority = otherRobot.GetPriority();
        RobotState otherState = otherRobot.currentState;

        if (myPriority > otherPriority)
        {
            return true;
        }
        else if (myPriority == otherPriority)
        {
            if (priorityStates.Contains(otherState))
            {
                return true;
            }
            if (robotId > otherRobot.robotId) return true;
        }

        return false;
    }

    private void PauseForSafety(Robot otherRobot, float distance)
    {
        /* float myRadius = transform.lossyScale.x / 2f;
        float otherRadius = otherRobot.transform.lossyScale.x / 2f;
        float safeDistance = 1f; */
        //Debug.Log($"[Robot {robotId}] Higher priority than Robot {otherRobot.robotId}. Maintaining safe distance {safeDistance}.");

        if (distance < safeDistanceThreshold)
        {
            isPausedForSafety = true;
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
                candidatePos = new Vector3(
                    Mathf.RoundToInt(candidatePos.x),
                    Mathf.RoundToInt(candidatePos.y),
                    Mathf.RoundToInt(candidatePos.z)
                );

                if (IsPositionValid(candidatePos) && IsSafeFromPath(candidatePos) && IsPathClear(otherRobotPos, candidatePos, otherRobot))
                {
                    if (Vector3.Distance(candidatePos, myPos) < Vector3.Distance(otherRobotPos, myPos))
                        continue;

                    yieldPos = candidatePos;
                    Debug.Log($"[Robot {robotId}] Found yield position for Robot {otherRobot.robotId} at {yieldPos} with distance {dist} and angle {angle}");
                    return true;
                }
            }
        }
        return false;
    }

    private bool IsSafeFromPath(Vector3 target)
    {
        if (pathQueue == null || pathQueue.Count == 0) return true;

        int pointsToCheck = Mathf.Min(pathQueue.Count, 10);
        var pathArray = pathQueue.ToArray();

        for (int i = 0; i < pointsToCheck; i++)
        {
            if (Vector3.Distance(target, pathArray[i]) < 1f) return false;
        }
        return true;
    }

    private bool IsPositionValid(Vector3 pos)
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

    private bool IsPathClear(Vector3 start, Vector3 end, Robot robot)
    {
        Vector3 direction = (end - start).normalized;
        float distance = Vector3.Distance(start, end);

        RaycastHit[] hits = Physics.SphereCastAll(start, perceptionRadius, direction, distance);

        foreach (var hit in hits)
        {
            if (hit.collider.gameObject == robot.gameObject) continue;
            if (hit.collider.gameObject == gameObject) return false;

            if (hit.collider.CompareTag("Invalid") || hit.collider.CompareTag("Robot"))
            {
                return false;
            }
        }
        return true;
    }

    private void HandleStaticObstacle(GameObject objectHit, float distance)
    {
        if (HandleSpecialObstacle(objectHit)) return;
        Vector3 target = pathQueue.Peek();
        float distanceToTarget = Vector3.Distance(transform.position, target);
        obstacleManager.ReportObstacle(objectHit, "unhandled");
        Vector3 direction = (target - transform.position).normalized;
        RaycastHit[] hits = Physics.SphereCastAll(transform.position, perceptionRadius, direction, distanceToTarget);
        foreach (var hit in hits)
        {
            if (hit.collider.gameObject == objectHit)
            {
                SendPathRequest();
                return;
            }
        }
    }

    private void ChargeAndCheck()
    {
        battery.ChargeRobot();
        if (battery.GetBattery() >= 100f)
        {
            Debug.Log($"[Robot {robotId}] Fully charged. Resuming tasks.");
            Vector3 closestDestination = GetClosestDestination();
            SetGoal(closestDestination);
            SetRobotVisibility(true);
            battery.SetChargeLock(false);
            SendPathRequest();
        }
    }

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

    protected Vector3 GetNextDestination()
    {
        Vector3 nextDestination = Vector3.zero;
        if (pathQueue.Count > 0) return nextDestination;
        if (destinations.Count > 0 /* && !isPathRequestPending */)
        {
            if (loop)
            {
                destinationIndex = (destinationIndex + 1) % destinations.Count;
            }
            else
            {
                if (destinationIndex >= destinations.Count - 1) return nextDestination;
                destinationIndex++;
            }
            nextDestination = destinations[destinationIndex];
        }
        return nextDestination;
    }

    private bool CheckIfChargingStationReached()
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

    private void CheckIfQueuedPointReached()
    {
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        if (Vector3.Distance(transform.position, target) < 0.02f)
        {
            pathQueue.Dequeue();
        }
    }

    private bool IsBlockingMyPath(Robot otherRobot)
    {
        Vector3 target = pathQueue.Peek();
        Vector3 direction = (target - transform.position).normalized;
        float distanceToTarget = Vector3.Distance(transform.position, target);
        if (direction == Vector3.zero) return false;

        RaycastHit[] hits = Physics.SphereCastAll(transform.position, perceptionRadius, direction, distanceToTarget);
        foreach (var hit in hits)
        {
            if (hit.collider.gameObject == otherRobot.gameObject) return true;
        }
        return false;
    }

    private void YieldBehavior()
    {
        if (FindBestReturnPoint(out Vector3 returnPos))
        {
            if (IsSafeToReturn(returnPos))
            {
                //yieldReturnPosition = returnPos;
                UpdatePathQueue(returnPos);
                currentState = RobotState.Moving;
                //isReturningFromYield = true;
            }
            else
            {
                yieldTimer += Time.deltaTime;
            }
        }
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

    private bool IsSafeToReturn(Vector3 targetPos)
    {
        /* Collider[] hits = Physics.OverlapSphere(targetPos, obstacleDistanceThreshold);
        foreach (var hit in hits)
        {
            if (hit.gameObject == gameObject) continue;
            if (hit.CompareTag("Robot")) return false;
        }
        return true; */
        Vector3 direction = (targetPos - transform.position).normalized;
        float distanceToTarget = Vector3.Distance(transform.position, targetPos);
        RaycastHit[] hits = Physics.SphereCastAll(transform.position, perceptionRadius, direction, distanceToTarget);
        foreach (var hit in hits)
        {
            if (hit.collider.gameObject == gameObject) continue;
            if (hit.collider.CompareTag("Robot")) return false;
        }
        return true;
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

    protected virtual void Move()
    {
        if (isPausedForSafety) return;
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        transform.position =
            Vector3.MoveTowards(transform.position, target, moveSpeed * Time.deltaTime);

        string currentNode = GetCurrentPositionNode();
        if (currentNode != lastNodeKey)
        {
            ObstacleGenerator.UpdateObstacleDirt(currentNode);
            lastNodeKey = currentNode;
        }

        battery.UpdateBattery(-moveSpeed * Time.deltaTime * 0.1f);
    }

    protected virtual bool HandleSpecialObstacle(GameObject objectHit)
    {
        return false;
    }

    protected virtual bool CheckDestinationReached()
    {
        if (isMovingToYield)
        {
            if (Vector3.Distance(transform.position, yieldTargetPosition) < 0.02f)
            {
                isMovingToYield = false;
                currentState = RobotState.Yielding;
                return true;
            }
        }
        return false;
    }

    protected virtual void UpdateTask()
    {
    }

    public virtual void OnObstacleUnhandled(GameObject obstacle)
    {
    }

    public virtual void OnObstacleHandled(int obstacleId)
    {
    }

    protected virtual int GetPriority() { return 0; }

    protected void SetGoal(Vector3 goalPos)
    {
        endX = goalPos.x;
        endY = goalPos.y;
        endZ = goalPos.z;
    }

    protected string GetCurrentPositionNode()
    {
        int x = (int)transform.position.x;
        int y = (int)transform.position.y;
        int z = (int)transform.position.z;
        return $"{x},{y - 1},{z}";
    }

    protected bool CannotHandleObstacle()
    {
        if (battery.GetBattery() <= 0f || isPathRequestPending || currentState == RobotState.Charging ||
            battery.IsChargeLocked() || currentState == RobotState.PerformingTask || currentState == RobotState.Yielding ||
            currentState == RobotState.WaitingForPath) return true;
        return false;
    }

    protected void SetRobotVisibility(bool visible)
    {
        foreach (var r in GetComponentsInChildren<Renderer>()) r.enabled = visible;
        foreach (var c in GetComponentsInChildren<Collider>()) c.enabled = visible;
        foreach (var c in GetComponentsInChildren<Canvas>()) c.enabled = visible;
    }

    protected Vector3 GetClosestDestination()
    {
        if (destinations.Count == 0) return Vector3.zero;
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
            return destinations[destinationIndex];
        }
        return Vector3.zero;
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

    protected void SendPathRequest()
    {
        if (isPathRequestPending || battery.IsChargeLocked()) return;
        if(endX == 0f && endY == 0f && endZ == 0f)
        {
            currentState = RobotState.Deadlock;
            Debug.LogWarning($"[Robot {robotId}] No valid goal set. Cannot send path request.");
            return;
        }

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
        if (isPathRequestPending || battery.IsChargeLocked()) return;
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

    private void PathResultCallback(PathPlannerResponseMsg res)
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
        // Charge lock remains until robot recharges battery.
        currentState = RobotState.Moving;
        //Debug.Log($"[Robot {robotId}] Received path with {res.path_x.Length} points.");
    }

    private void RobotCoordinationCallback(StringMsg msg)
    {
        RobotCoordination data = JsonUtility.FromJson<RobotCoordination>(msg.data);
        if (data.target_robot_id != robotId) return;
        if (data.command == "yield")
        {
            Vector3 targetPos = new(data.x, data.y, data.z);
            //if (yieldTargetPosition == targetPos) return;
            //Debug.Log($"[Robot {robotId}] Received yield command to {targetPos}");

            //currentState = RobotState.Yielding;

            Vector3[] existingPath = pathQueue.ToArray();
            pathQueue.Clear();
            pathQueue.Enqueue(targetPos);

            if (!isMovingToYield)
            {
                pathQueue.Enqueue(transform.position);
                foreach (var point in existingPath)
                {
                    pathQueue.Enqueue(point);
                }
            }
            else
            {
                for (int i = 1; i < existingPath.Length; i++)
                {
                    pathQueue.Enqueue(existingPath[i]);
                }
            }

            yieldTargetPosition = targetPos;
            //yieldReturnPosition = transform.position;
            isMovingToYield = true;
            //isReturningFromYield = false;
            isPausedForSafety = false;
            currentState = RobotState.Moving;
        }
    }
}
