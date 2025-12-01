using UnityEngine;
using System.Collections;

public class SecurityRobot : Robot
{
    private Vector3 securedLocation = new(6, 11, 0);

    protected override int GetPriority()
    {
        return 2;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
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
        obstacle.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);

        ReportObstacle(obstacle, "handled");
        endX = securedLocation.x;
        endY = securedLocation.y;
        SendRequest();
    }

    private IEnumerator DestroyUnattendedRoutine()
    {
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

        currentState = RobotState.Moving;
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