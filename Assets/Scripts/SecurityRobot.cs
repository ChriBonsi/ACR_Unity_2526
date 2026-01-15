using UnityEngine;
using System.Collections;
using RosMessageTypes.Std;
using System.Collections.Generic;

public class SecurityRobot : Robot
{
    private class SecurityBid
    {
        public int robotId;
        public int objectId;
        public float distance;
    }

    private readonly Vector3[] securedLocations = new Vector3[]
    {
        new(2, 0, 27),
        new(27, 0, 2),
    };
    private Vector3 securedLocation;
    private bool isDestroying = false;
    private bool isHoldingObstacle = false;
    private GameObject unattendedTarget;
    private readonly Dictionary<int, SecurityBid> bestBids = new();

    new void Start()
    {
        base.Start();
        ros.Subscribe<StringMsg>("security_robot/security_coordination", CoordinationCallback);
    }

    new void Update()
    {
        base.Update();
        if(currentState == RobotState.Moving) CheckTasksToDo();
    }

    protected override int GetPriority()
    {
        return 3;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        if(isHoldingObstacle || CannotHandleObstacle()) return false;
        if (objectHit.CompareTag("UnattendedObstacle"))
        {
            ProposeBid(objectHit);
            currentState = RobotState.PerformingTask;
            unattendedTarget = objectHit;
            obstacleManager.ReportObstacle(gameObject, "unhandled");
            pathQueue.Clear();
            pathQueue.Enqueue(objectHit.transform.position);
            return true;
        }
        return false;
    }

    protected override void UpdateTask()
    {
        if (currentState != RobotState.PerformingTask || unattendedTarget == null) return;

        if (Vector3.Distance(transform.position, unattendedTarget.transform.position) < 0.1f)
        {
            if (!isHoldingObstacle)
            {
                isHoldingObstacle = true;
                StartCoroutine(PickupRoutine(unattendedTarget));
            }
        }
        else
        {
            Move();
        }
    }

    public override void OnObstacleUnhandled(GameObject obstacle)
    {
        if (CannotHandleObstacle()) return;
        if (!obstacle.CompareTag("UnattendedObstacle")) return;
        ProposeBid(obstacle);
    }

    public override void OnObstacleHandled(int obstacleId)
    {
        if (bestBids.ContainsKey(obstacleId))
        {
            bestBids.Remove(obstacleId);
        }
        IfIWasHandlingIt(obstacleId);
    }

    private void CheckTasksToDo()
    {
        if (bestBids.Count == 0 || isHoldingObstacle || CannotHandleObstacle()) return;

        SecurityBid bestTask = null;
        float closestDistance = float.MaxValue;

        foreach (var bid in bestBids.Values)
        {
            if (bid.robotId == robotId)
            {
                if (bid.distance < closestDistance)
                {
                    closestDistance = bid.distance;
                    bestTask = bid;
                }
            }
        }

        if (bestTask != null && unattendedTarget == null)
        {
            GameObject obstacle = obstacleManager.GetObstacle(bestTask.objectId);
            if (obstacle != null)
            {
                Debug.Log($"[SecurityRobot {robotId}] Executing winning bid for obstacle {bestTask.objectId}.");
                unattendedTarget = obstacle;
                pathQueue.Clear();
                Vector3 goal = obstacle.transform.position;
                SetGoal(goal);
                SendPathRequest();
            }
            else
            {
                bestBids.Remove(bestTask.objectId);
            }
        }
    }

    private void IfIWasHandlingIt(int obstacleId)
    {
        if (unattendedTarget != null && unattendedTarget.GetInstanceID() == obstacleId && !isHoldingObstacle)
        {
            icon.SetActive(false);
            unattendedTarget = null;
            isHoldingObstacle = false;
            isDestroying = false;
            obstacleManager.ReportObstacle(gameObject, "handled");
            Vector3 closestDestination = GetClosestDestination();
            SetGoal(closestDestination);
            SendPathRequest();
        }
    }

    private void ProposeBid(GameObject obstacle)
    {
        if(obstacle == null) return;
        int objectId = obstacle.GetInstanceID();
        float myDistance = Vector3.Distance(transform.position, obstacle.transform.position);

        SecurityBid myBid = new()
        {
            robotId = robotId,
            objectId = objectId,
            distance = myDistance
        };

        bool isBetter = false;
        if (bestBids.TryGetValue(objectId, out SecurityBid currentBest))
        {
            if (IsBidBetter(myBid, currentBest))
            {
                isBetter = true;
            }
        }
        else
        {
            isBetter = true;
        }

        if (isBetter)
        {
            bestBids[objectId] = myBid;
            PublishBid(myBid);
        }
    }

    private bool IsBidBetter(SecurityBid newBid, SecurityBid currentBid)
    {
        if (newBid.distance < currentBid.distance/*  - 0.1f */) return true;
        if (newBid.distance > currentBid.distance/*  + 0.1f */) return false;
        return newBid.robotId < currentBid.robotId;
    }

    private void PublishBid(SecurityBid bid)
    {
        string taskJson = JsonUtility.ToJson(bid);
        ros.Publish("security_robot/security_coordination", new StringMsg() { data = taskJson });
    }

    private void SetClosestSecuredLocation()
    {
        float minDistance = float.PositiveInfinity;
        Vector3 closest = Vector3.zero;

        for (int i = 0; i < securedLocations.Length; i++)
        {
            Vector3 location = securedLocations[i];
            location.y += 1f;
            float distance = Vector3.Distance(transform.position, location);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = location;
            }
        }

        securedLocation = closest;
        SetGoal(securedLocation);
    }

    private IEnumerator PickupRoutine(GameObject obstacle)
    {
        currentState = RobotState.PerformingTask;
        Debug.Log($"[SecurityRobot {robotId}] Clearing unattended obstacle {obstacle.GetInstanceID()}...");

        icon.SetActive(true);
        yield return new WaitForSeconds(2f);
        icon.SetActive(false);

        obstacle.GetComponent<Collider>().enabled = false;

        obstacle.transform.SetParent(transform);
        obstacle.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
        obstacle.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);

        obstacleManager.ReportObstacle(obstacle, "handled");
        obstacleManager.ReportObstacle(gameObject, "handled");
        SetClosestSecuredLocation();
        Debug.Log($"[SecurityRobot {robotId}] Securing obstacle {obstacle.GetInstanceID()} at location {securedLocation}.");
        SendPathRequest();
    }

    protected override bool CheckDestinationReached()
    {
        if(base.CheckDestinationReached()) return true;
        if (isHoldingObstacle && Vector3.Distance(transform.position, securedLocation) < 0.1f)
        {
            if (!isDestroying) StartCoroutine(DestroyUnattendedRoutine());
            return true;
        }
        return false;
    }

    private IEnumerator DestroyUnattendedRoutine()
    {
        currentState = RobotState.PerformingTask;
        isDestroying = true;

        icon.SetActive(true);
        yield return new WaitForSeconds(2f);
        icon.SetActive(false);

        Destroy(unattendedTarget);
        Debug.Log($"[SecurityRobot {robotId}] Package destroyed at {securedLocation}.");

        isHoldingObstacle = false;
        isDestroying = false;
        unattendedTarget = null;
        currentState = RobotState.Moving;
        Vector3 closestDestination = GetClosestDestination();
        SetGoal(closestDestination);
    }

    private void CoordinationCallback(StringMsg msg)
    {
        SecurityBid incomingBid = JsonUtility.FromJson<SecurityBid>(msg.data);
        if (incomingBid.robotId == robotId) return;

        int objectId = incomingBid.objectId;

        if (obstacleManager.GetObstacle(objectId) == null)
        {
            if (bestBids.ContainsKey(objectId))
            {
                bestBids.Remove(objectId);
            }
            return;
        }

        if (bestBids.TryGetValue(objectId, out SecurityBid currentBest))
        {
            if (IsBidBetter(incomingBid, currentBest))
            {
                bestBids[objectId] = incomingBid;

                if (currentBest.robotId == robotId)
                {
                    Debug.Log($"[SecurityRobot {robotId}] Outbid by Robot {incomingBid.robotId} for obstacle {objectId}.");
                    IfIWasHandlingIt(objectId);
                }
            }
            else if (currentBest.robotId == robotId)
            {
                PublishBid(currentBest);
            }
        }
        else
        {
            bestBids[objectId] = incomingBid;
            ProposeBid(obstacleManager.GetObstacle(objectId));
        }
    }
}