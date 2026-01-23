using UnityEngine;

public class BaggageRobot : Robot
{
    private float capacity = 0;
    private readonly int maxCapacity = 20;
    private bool loading = true;

    protected override int GetPriority()
    {
        return 1;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        return false;
    }

    protected override bool CheckDestinationReached()
    {
        if (base.CheckDestinationReached()) return true;
        if (Vector3.Distance(transform.position, new Vector3(endX, endY, endZ)) < 0.1f)
        {
            currentState = RobotState.PerformingTask;
            pathQueue.Clear();
            return true;
        }
        return false;
    }

    protected override void UpdateTask()
    {
        if (currentState != RobotState.PerformingTask) return;

        battery.ChargeRobot();

        if (loading)
        {
            if (capacity < maxCapacity)
            {
                capacity += Time.deltaTime * 5;
                SetRobotVisibility(false);
            }
            else
            {
                icon.SetActive(true);
                //Debug.Log($"[BaggageRobot {robotId}] Fully loaded. Resuming movement.");
                if (SimulationLogger.Instance != null)
                    SimulationLogger.Instance.LogEvent("BaggageRobot", robotId.ToString(), "LoadingComplete", $"Capacity: {maxCapacity}");
                TriggerBackAndForth();
            }
        }
        else
        {
            if (capacity > 0)
            {
                capacity -= Time.deltaTime * 5;
                SetRobotVisibility(false);
            }
            else
            {
                icon.SetActive(false);
                //Debug.Log($"[BaggageRobot {robotId}] Unloaded baggage. Resuming movement.");
                if (SimulationLogger.Instance != null)
                    SimulationLogger.Instance.LogEvent("BaggageRobot", robotId.ToString(), "UnloadingComplete");
                TriggerBackAndForth();
            }
        }
    }

    private void TriggerBackAndForth()
    {
        loading = !loading;
        SetRobotVisibility(true);
        Vector3 nextDestination = GetNextDestination();
        if (nextDestination != Vector3.zero)
        {
            SetGoal(nextDestination);
            SendPathRequest();
        }
    }
}