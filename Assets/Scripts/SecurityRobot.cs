using UnityEngine;
using System.Collections;

public class SecurityRobot : Robot
{
    private readonly Vector3[] securedLocations = new Vector3[]
    {
        new(2, 27),
        new(27, 2),
    };
    private Vector3 securedLocation;
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

    private void GetClosestSecuredLocation()
    {
        float minDistance = float.PositiveInfinity;
        Vector3 closest = Vector3.zero;

        foreach (var location in securedLocations)
        {
            float distance = Vector3.Distance(transform.position, location);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = location;
            }
        }

        securedLocation = closest;
    }

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
        obstacle.transform.localScale = new Vector3(0.8f, 0.8f, 1f);
        obstacle.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);

        obstacleManager.ReportObstacle(obstacle, "handled");
        GetClosestSecuredLocation();
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