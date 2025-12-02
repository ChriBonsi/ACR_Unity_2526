using UnityEngine;
using System.Collections;

public class CleanerRobot : Robot
{
    protected override int GetPriority()
    {
        return 1;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        if (objectHit.CompareTag("DirtObstacle"))
        {
            if (Vector3.Distance(transform.position, objectHit.transform.position) < 0.1f)
            {
                if (currentState != RobotState.HandlingObstacle)
                {
                    StartCoroutine(CleanDirtRoutine(objectHit));
                }
            }
            return true;
        }
        return false;
    }

    protected override void UpdateTask()
    {
    }

    /* protected override bool IsTaskInterruptible()
    {
        return true; 
    } */

    private IEnumerator CleanDirtRoutine(GameObject obstacle)
    {
        Debug.Log($"[CleanerRobot {robotId}] Cleaning obstacle {obstacle.GetInstanceID()}...");
        currentState = RobotState.HandlingObstacle;

        icon.SetActive(true);

        yield return new WaitForSeconds(2f);

        icon.SetActive(false);

        Destroy(obstacle);
        ReportObstacle(obstacle, "handled");

        currentState = RobotState.Moving;
    }
}