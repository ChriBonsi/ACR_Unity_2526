using UnityEngine;
using System.Collections;

public class SecurityRobot : Robot
{
    private readonly Vector3[] securedLocations = new Vector3[]
    {
        new(2, 0, 27),
        new(27, 0, 2),
    };
    private Vector3 securedLocation;
    private bool isDestroying = false;
    private bool isHoldingObstacle = false;
    private GameObject unattendedTarget;

    protected override int GetPriority()
    {
        return 3;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        if (isHoldingObstacle) return false;
        if (objectHit.CompareTag("UnattendedObstacle"))
        {
            currentState = RobotState.PerformingTask;
            unattendedTarget = objectHit;
            pathQueue.Clear();
            pathQueue.Enqueue(objectHit.transform.position);
            return true;
        }
        return false;
    }

    protected override void UpdateTask()
    {
        if (currentState != RobotState.PerformingTask || unattendedTarget == null) return;

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
                if (CheckIfChargingStationReached()) return;
                CheckIfQueuedPointReached();
            }
            return;
        }

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

    private void GetClosestSecuredLocation()
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
    }

    private IEnumerator PickupRoutine(GameObject obstacle)
    {
        currentState = RobotState.PerformingTask;
        pathQueue.Clear();
        Debug.Log($"[SecurityRobot {robotId}] Clearing unattended obstacle {obstacle.GetInstanceID()}...");

        icon.SetActive(true);
        yield return new WaitForSeconds(2f);
        icon.SetActive(false);

        obstacle.GetComponent<Collider>().enabled = false;

        obstacle.transform.SetParent(transform);
        obstacle.transform.localScale = new Vector3(0.8f, 0.8f, 1f);
        obstacle.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);

        obstacleManager.ReportObstacle(obstacle, "handled");
        GetClosestSecuredLocation();
        endX = securedLocation.x;
        endY = securedLocation.y;
        endZ = securedLocation.z;
        queueBackTaskState = true;
        SendRequest();
    }

    private IEnumerator DestroyUnattendedRoutine()
    {
        isDestroying = true;

        icon.SetActive(true);
        yield return new WaitForSeconds(2f);
        icon.SetActive(false);

        Destroy(unattendedTarget);
        Debug.Log($"[SecurityRobot {robotId}] Package destroyed at {securedLocation}.");

        isHoldingObstacle = false;
        isDestroying = false;
        queueBackTaskState = false;
        unattendedTarget = null;
        currentState = RobotState.Moving;
        SetNextClosestDestination();
    }
}