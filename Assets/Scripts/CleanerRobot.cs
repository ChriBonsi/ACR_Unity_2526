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
                obstacleDetected = true;
            }
            return true;
        }

        return false;
    }

    private IEnumerator CleanDirtRoutine(GameObject dirt)
    {
        isPerformingTask = true;
        Debug.Log($"[CleanerRobot {robotId}] Cleaning dirt at {dirt.transform.position}...");
        
        yield return new WaitForSeconds(2f);
        
        if (dirt != null) Destroy(dirt);
        
        isPerformingTask = false;
        //obstacleDetected = false; 
    }
}