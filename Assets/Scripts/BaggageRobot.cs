using UnityEngine;

public class BaggageRobot : Robot
{
    public float capacity = 0;
    int maxCapacity = 20;
    public bool loading = true;

    protected override int GetPriority()
    {
        return 1;
    }

    protected override bool HandleSpecialObstacle(GameObject objectHit)
    {
        return false;
    }

    protected override void UpdateTask()
    {
        if(currentState != RobotState.PerformingTask) return;

        UpdateBattery(chargeRate * Time.deltaTime);

        if(loading){
            if(capacity < maxCapacity)
            {
                capacity += Time.deltaTime * 5;
            }
            else{
                Debug.Log($"[BaggageRobot {robotId}] Fully loaded. Resuming movement.");
                loading = false;
                queueBackTaskState = false;
                if(pathQueue.Count == 0) CheckAndAskForNewPath();
                else currentState = RobotState.Moving;
            }
        }
        else{
            if(capacity > 0)
            {
                capacity -= Time.deltaTime * 5;
            }
            else{
                Debug.Log($"[BaggageRobot {robotId}] Unloaded baggage. Resuming movement.");
                loading = true;
                queueBackTaskState = false;
                if(pathQueue.Count == 0) CheckAndAskForNewPath();
                else currentState = RobotState.Moving;
            }
        }
    }
}