using UnityEngine;
using System.Collections;

public class CleanerRobot : Robot
{
    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        if (objectHit.CompareTag("DirtObstacle"))
        {
            if (Vector3.Distance(transform.position, objectHit.transform.position) < 0.1f)
            {
                if (!isPerformingTask)
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

    private IEnumerator CleanDirtRoutine(GameObject obstacle)
    {
        isPerformingTask = true;
        Debug.Log($"[CleanerRobot {robotId}] Cleaning obstacle {obstacle.GetInstanceID()}...");

        icon.SetActive(true);

        yield return new WaitForSeconds(2f);

        icon.SetActive(false);

        Destroy(obstacle);
        ReportObstacle(obstacle, "handled");
        isPerformingTask = false;
    }
}