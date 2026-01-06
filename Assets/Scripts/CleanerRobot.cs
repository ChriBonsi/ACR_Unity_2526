using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Std;
using System;

public class CleanerRobot : Robot
{
    [Serializable]
    public class CleaningBid
    {
        public int robotId;
        public int objectId;
        public float distance;
    }

    [Serializable]
    public class KeyValuePair
    {
        public int key;
        public CleaningBid val;
    }

    private GameObject cleaningTarget;
    private bool isCleaning = false;
    private readonly Dictionary<int, CleaningBid> bestBids = new();
    public List<KeyValuePair> MyList = new();

    new void Start()
    {
        base.Start();
        ros.Subscribe<StringMsg>("cleaner_robot/cleaning_coordination", CoordinationCallback);
    }

    new void Update()
    {
        base.Update();
        MyList.Clear();
        foreach (var kvp in bestBids)
        {
            MyList.Add(new KeyValuePair { key = kvp.Key, val = kvp.Value });
        }
    }

    protected override int GetPriority()
    {
        return 2;
    }

    protected override void Move()
    {
        if (isPausedForSafety) return;
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        transform.position =
            Vector3.MoveTowards(transform.position, target, moveSpeed * Time.deltaTime);

        string currentNode = GetCurrentPositionNode();
        if (currentNode != lastNodeKey)
        {
            ObstacleGenerator.CleanedDirt(currentNode);
            lastNodeKey = currentNode;
        }

        battery.UpdateBattery(-moveSpeed * Time.deltaTime * 0.1f);
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        //if(battery.GetBattery() <= 0f || currentState == RobotState.Charging || battery.IsChargeLocked()) return false;
        if (CannotHandleObstacle()) return false;
        if (objectHit.CompareTag("DirtObstacle"))
        {
            int objectId = objectHit.GetInstanceID();
            CleaningBid myBid = new()
            {
                robotId = robotId,
                objectId = objectId,
                distance = 0f
            };

            if (bestBids.TryGetValue(objectId, out CleaningBid currentBest))
            {
                if (!IsBidBetter(myBid, currentBest)) return false;
            }

            bestBids[objectId] = myBid;
            PublishBid(myBid);

            currentState = RobotState.PerformingTask;
            cleaningTarget = objectHit;
            pathQueue.Clear();
            pathQueue.Enqueue(objectHit.transform.position);
            return true;
        }
        return false;
    }

    protected override void UpdateTask()
    {
        if (currentState != RobotState.PerformingTask || cleaningTarget == null) return;

        if (CheckIfTouchingTarget(cleaningTarget))
        {
            if (!isCleaning)
            {
                gameObject.GetComponent<BoxCollider>().enabled = false;
                isCleaning = true;
                StartCoroutine(CleanDirtRoutine(cleaningTarget));
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
        if (!obstacle.CompareTag("DirtObstacle")) return;
        ProposeBid(obstacle);
    }

    public override void OnObstacleHandled(int obstacleId)
    {
        if (bestBids.ContainsKey(obstacleId))
        {
            bestBids.Remove(obstacleId);
        }
        if (CannotHandleObstacle()) return;
        /* if (cleaningTarget != null && cleaningTarget.GetInstanceID() == obstacleId)
        {
            if (!isCleaning)
            {
                cleaningTarget = null;
                CheckTasksToDo();
                currentState = RobotState.Moving;
                SendPathRequest();
            }
        } */
        if (!isCleaning) IfIWasDoingIt(obstacleId);
    }

    private bool CheckIfTouchingTarget(GameObject target)
    {
        Collider targetCollider = target.GetComponent<Collider>();
        Collider robotCollider = GetComponent<Collider>();
        return robotCollider.bounds.Intersects(targetCollider.bounds);
    }

    private IEnumerator CleanDirtRoutine(GameObject obstacle)
    {
        Debug.Log($"[CleanerRobot {robotId}] Cleaning obstacle {obstacle.GetInstanceID()}...");
        currentState = RobotState.PerformingTask;
        obstacle.GetComponent<Collider>().enabled = false;

        icon.SetActive(true);
        yield return new WaitForSeconds(2f);
        icon.SetActive(false);

        obstacleManager.ReportObstacle(obstacle, "handled");
        isCleaning = false;
        cleaningTarget = null;

        yield return new WaitForSeconds(0.2f);

        Destroy(obstacle);
        ObstacleGenerator.CleanedDirt(GetCurrentPositionNode());
        gameObject.GetComponent<BoxCollider>().enabled = true;

        currentState = RobotState.Moving;

        if (!CannotHandleObstacle()) { CheckTasksToDo(); }
        else
        {
            Vector3 closestDestination = GetClosestDestination();
            SetGoal(closestDestination);
        }
        SendPathRequest();
    }

    private void CheckTasksToDo()
    {
        if (bestBids.Count == 0) return;

        CleaningBid bestTask = null;
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

        if (bestTask != null && cleaningTarget == null)
        {
            GameObject obstacle = obstacleManager.GetObstacle(bestTask.objectId);
            if (obstacle != null)
            {
                Debug.Log($"[CleanerRobot {robotId}] Executing winning bid for obstacle {bestTask.objectId}.");
                pathQueue.Clear();
                Vector3 goal = obstacle.transform.position;
                SetGoal(goal);
            }
            else
            {
                bestBids.Remove(bestTask.objectId);
            }
        }
        else
        {
            Vector3 closestDestination = GetClosestDestination();
            SetGoal(closestDestination);
        }
    }

    private void IfIWasDoingIt(int obstacleId)
    {
        if (cleaningTarget != null && cleaningTarget.GetInstanceID() == obstacleId)
        {
            cleaningTarget = null;
            isCleaning = false;
            CheckTasksToDo();
            currentState = RobotState.Moving;
            SendPathRequest();
        }
    }

    private void ProposeBid(GameObject obstacle)
    {
        int objectId = obstacle.GetInstanceID();
        float myDistance = Vector3.Distance(transform.position, obstacle.transform.position);

        CleaningBid myBid = new()
        {
            robotId = robotId,
            objectId = objectId,
            distance = myDistance
        };

        bool isBetter = false;
        if (bestBids.TryGetValue(objectId, out CleaningBid currentBest))
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
            CheckTasksToDo();
            currentState = RobotState.Moving;
            SendPathRequest();
        }
    }

    private bool IsBidBetter(CleaningBid newBid, CleaningBid currentBid)
    {
        if (newBid.distance < currentBid.distance/*  - 0.1f */) return true;
        if (newBid.distance > currentBid.distance/*  + 0.1f */) return false;
        return newBid.robotId < currentBid.robotId;
    }

    private void PublishBid(CleaningBid bid)
    {
        string taskJson = JsonUtility.ToJson(bid);
        ros.Publish("cleaner_robot/cleaning_coordination", new StringMsg() { data = taskJson });
    }

    private void CoordinationCallback(StringMsg msg)
    {
        CleaningBid incomingBid = JsonUtility.FromJson<CleaningBid>(msg.data);
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

        if (bestBids.TryGetValue(objectId, out CleaningBid currentBest))
        {
            if (IsBidBetter(incomingBid, currentBest))
            {
                bestBids[objectId] = incomingBid;

                if (currentBest.robotId == robotId)
                {
                    Debug.Log($"[CleanerRobot {robotId}] Outbid by Robot {incomingBid.robotId} for obstacle {objectId}.");
                    /* if (cleaningTarget != null && cleaningTarget.GetInstanceID() == objectId)
                    {
                        cleaningTarget = null;
                        isCleaning = false;
                        CheckTasksToDo();
                        currentState = RobotState.Moving;
                        SendPathRequest();
                    } */
                    if (CannotHandleObstacle()) return;
                    IfIWasDoingIt(objectId);
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
        }
    }
}