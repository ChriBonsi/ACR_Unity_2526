using UnityEngine;
using System.Collections;

public class SecurityRobot : Robot
{
    private Vector3 securedLocation = new(6, 11, 0);
    private bool isDestroying = false;
    private bool isHoldingObstacle = false;

    protected override int GetPriority()
    {
        return 2;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        if(currentState == RobotState.HandlingObstacle) return false;

        if (objectHit.CompareTag("UnattendedObstacle"))
        {
            if (Vector3.Distance(transform.position, objectHit.transform.position) < 0.2f)
            {
                if (currentState != RobotState.HandlingObstacle)
                {
                    StartCoroutine(PickupRoutine(objectHit));
                }
            }
            return true;
        }
        return false;
    }

    protected override void UpdateTask()
    {
        if (currentState != RobotState.HandlingObstacle) return;

        if (isHoldingObstacle)
        {
            CheckSensors();

            if (Vector3.Distance(transform.position, securedLocation) < 0.1f)
            {
                if (!isDestroying) StartCoroutine(DestroyUnattendedRoutine());
            }
            else
            {
                Move();
                CheckIfQueuedPointReached();
            }
        }
    }

    /* protected override bool IsTaskInterruptible()
    {
        return !isHoldingObstacle; 
    } */

    private IEnumerator PickupRoutine(GameObject obstacle)
    {
        currentState = RobotState.HandlingObstacle;
        pathQueue.Clear();
        Debug.Log($"[SecurityRobot {robotId}] Clearing unattended obstacle {obstacle.GetInstanceID()}...");

        icon.SetActive(true);

        yield return new WaitForSeconds(2f);

        icon.SetActive(false);

        obstacle.GetComponent<BoxCollider2D>().enabled = false;

        obstacle.transform.SetParent(transform);
        obstacle.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);

        ReportObstacle(obstacle, "handled");
        isHoldingObstacle = true;
        endX = securedLocation.x;
        endY = securedLocation.y;
        queueBackTaskState = true;
        SendRequest();
    }

    private IEnumerator DestroyUnattendedRoutine()
    {
        isDestroying = true;

        icon.SetActive(true);

        yield return new WaitForSeconds(2f);

        icon.SetActive(false);

        foreach (Transform child in transform)
        {
            if (child.CompareTag("UnattendedObstacle"))
            {
                Destroy(child.gameObject);
            }
        }

        Debug.Log($"[SecurityRobot {robotId}] Package destroyed at {securedLocation}.");

        isHoldingObstacle = false;
        isDestroying = false;
        queueBackTaskState = false;
        currentState = RobotState.Moving;
    }
}