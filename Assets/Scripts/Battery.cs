using UnityEngine;

public class Battery
{
    private const float DEFAULT_CHARGE_RATE = 20f;
    private const float DEFAULT_BATTERY = 100f;

    private readonly int robotId;
    private bool chargeLock = false;
    private readonly float chargeRate;
    private float battery;

    public Battery(int robotId) : this(robotId, DEFAULT_BATTERY, DEFAULT_CHARGE_RATE)
    {
    }

    public Battery(int robotId, float battery) : this(robotId, battery, DEFAULT_CHARGE_RATE)
    {
    }

    public Battery(int robotId, float battery, float chargeRate)
    {
        this.robotId = robotId; 
        this.battery = battery;
        this.chargeRate = chargeRate;
    }

    public float GetBattery()
    {
        return battery;
    }

    public bool IsChargeLocked()
    {
        return chargeLock;
    }

    public void ChargeRobot()
    {
        UpdateBattery(chargeRate * Time.deltaTime);
    }

    public void SetChargeLock(bool value)
    {
        chargeLock = value;
    }

    public void UpdateBattery(float amount)
    {
        battery = Mathf.Clamp(battery + amount, 0f, 100f);
    }
}