using UnityEngine;
using System.Collections;
using RosMessageTypes.Std;
using System;
using System.Collections.Generic;

public class CleanerRobot : Robot
{
    [Serializable]
    private class CleaningBid
    {
        public int robotId;
        public int objectId;
        public float distance;
    }

    private GameObject cleaningTarget;
    private bool isCleaning = false;
    private readonly Dictionary<int, CleaningBid> cleaningBids = new();
    
    new void Start()
    {
        base.Start();
        ros.Subscribe<StringMsg>("cleaner_robot/cleaning_coordination", CleaningBidCoordinationCallback);
    }
    
    protected override int GetPriority()
    {
        return 2;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        if (objectHit.CompareTag("DirtObstacle"))
        {
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
        if(currentState != RobotState.PerformingTask || cleaningTarget == null) return;

        //if (Vector3.Distance(transform.position, cleaningTarget.transform.position) < 0.1f)
        if(CheckIfTouchingTarget(cleaningTarget))
        {
            if(!isCleaning)
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

        icon.SetActive(true);
        yield return new WaitForSeconds(2f);
        icon.SetActive(false);
        
        obstacleManager.ReportObstacle(obstacle, "handled");
        isCleaning = false;
        cleaningTarget = null;
        Destroy(obstacle);

        yield return new WaitForSeconds(0.2f);

        gameObject.GetComponent<BoxCollider>().enabled = true;
        currentState = RobotState.Moving;
        SendPathRequest();
    }

    private void CleaningBidCoordinationCallback(StringMsg msg)
    {
        CleaningBid data = JsonUtility.FromJson<CleaningBid>(msg.data);
        if (data == null) return;
    }
}