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

    protected override void UpdateTask()
    {
        if(currentState != RobotState.PerformingTask) return;

        battery.ChargeRobot();

        if(loading){
            if(capacity < maxCapacity)
            {
                capacity += Time.deltaTime * 5;
                SetRobotVisibility(false);
            }
            else{
                //Debug.Log($"[BaggageRobot {robotId}] Fully loaded. Resuming movement.");
                loading = false;
                SetRobotVisibility(true);
                if(pathQueue.Count == 0) CheckAndAskForNewPath();
                else currentState = RobotState.Moving;
            }
        }
        else{
            if(capacity > 0)
            {
                capacity -= Time.deltaTime * 5;
                SetRobotVisibility(false);
            }
            else{
                //Debug.Log($"[BaggageRobot {robotId}] Unloaded baggage. Resuming movement.");
                loading = true;
                SetRobotVisibility(true);
                if(pathQueue.Count == 0) CheckAndAskForNewPath();
                else currentState = RobotState.Moving;
            }
        }
    }
}