using UnityEngine;
using System.Collections;

public class SecurityRobot : Robot
{
    private Vector3 securedLocation = new(6, 11, 0);
    private bool isDestroying = false;
    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        if (objectHit.CompareTag("UnattendedObstacle"))
        {
            if (Vector3.Distance(transform.position, objectHit.transform.position) < 0.2f)
            {
                if (!isPerformingTask)
                {
                    StartCoroutine(PickupRoutine(objectHit));
                }
            }
            return true;
        }

        return false;
    }

    private IEnumerator PickupRoutine(GameObject obstacle)
    {
        isPerformingTask = true;
        pathQueue.Clear();
        Debug.Log($"[SecurityRobot {robotId}] Clearing unattended obstacle {obstacle.GetInstanceID()}...");

        icon.SetActive(true);

        yield return new WaitForSeconds(2f);

        icon.SetActive(false);

        obstacle.GetComponent<BoxCollider2D>().enabled = false;


        obstacle.transform.SetParent(transform);
        obstacle.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);


        ReportObstacle(obstacle, "handled");
        endX = securedLocation.x;
        endY = securedLocation.y;
        SendRequest();
    }

    protected override void UpdateTask()
    {
        if (!isPerformingTask) return;
        CheckForObstacles();

        if (obstacleDetected)
        {
            SendRequest();
            return;
        }

        if (Vector3.Distance(transform.position, securedLocation) < 0.2f)
        {
            if (!isDestroying) StartCoroutine(DestroyUnattendedRoutine());
            return;
        }

        Move();
        CheckIfQueuedPointReached();
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

        //SetNextDestination();
        isPerformingTask = false;
        obstacleDetected = false;
        isDestroying = false;
    }

    private void SetNextDestination()
    {
        float minDistance = float.MaxValue;
        int closestIndex = -1;
        for (int i = 0; i < destinations.Count; i++)
        {
            float d = Vector3.Distance(transform.position, destinations[i]);
            if (d < minDistance)
            {
                minDistance = d;
                closestIndex = i;
            }
        }
        if (closestIndex != -1) destinationIndex = closestIndex;
    }
}