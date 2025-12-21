using UnityEngine;
using System.Collections;

public class CleanerRobot : Robot
{
    private GameObject cleaningTarget;
    private bool isCleaning = false;
    
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

        currentState = RobotState.Moving;
        SendRequest();
    }
}