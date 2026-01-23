using UnityEngine;
using System.IO;
using System.Text;
using System.Collections;

public class SimulationLogger : MonoBehaviour
{
    [Header("Logging Settings")]
    [Tooltip("Log data every N seconds")]
    public float logInterval = 0.2f;
    public bool enableLogging = true;
    public string logFolder = "Logs";

    public static SimulationLogger Instance { get; private set; }

    private string sessionPath;
    private string robotLogPath;
    private string eventLogPath;
    
    private Robot[] robots;

    void Awake()
    {
        if (Instance == null) Instance = this;
        else Destroy(gameObject);
    }

    void Start()
    {
        if (!enableLogging) return;

        string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string rootLogPath = Path.Combine(Application.dataPath, "../" + logFolder);
        sessionPath = Path.Combine(rootLogPath, $"Session_{timestamp}");

        if (!Directory.Exists(sessionPath))
        {
            Directory.CreateDirectory(sessionPath);
        }

        robotLogPath = Path.Combine(sessionPath, "robots.csv");
        eventLogPath = Path.Combine(sessionPath, "events.csv");

        File.WriteAllText(robotLogPath, "Time,RobotID,Type,X,Y,Z,Battery,State,IsPaused\n");
        File.WriteAllText(eventLogPath, "Time,AgentType,AgentID,Event,Details\n");

        Debug.Log($"[SimulationLogger] Logging to: {sessionPath}");
        
        StartCoroutine(LoggingRoutine());
    }

    public void LogEvent(string agentType, string agentId, string eventName, string details = "")
    {
        if (!enableLogging) return;
        string line = $"{Time.time:F2},{agentType},{agentId},{eventName},{details}\n";
        File.AppendAllText(eventLogPath, line);
    }

    IEnumerator LoggingRoutine()
    {
        while (enableLogging)
        {
            yield return new WaitForSeconds(logInterval);
            LogStep();
        }
    }

    void LogStep()
    {
        float currentTime = Time.time;

        robots = FindObjectsByType<Robot>(FindObjectsSortMode.None);
        if (robots != null)
        {
            StringBuilder sb = new();
            foreach (var robot in robots)
            {
                if (robot == null) continue;
                
                string line = string.Format("{0:F2},{1},{2},{3:F3},{4:F3},{5:F3},{6:F1},{7},{8}",
                    currentTime,
                    robot.robotId,
                    robot.robotType,
                    robot.transform.position.x,
                    robot.transform.position.y,
                    robot.transform.position.z,
                    robot.CurrentBattery,
                    robot.currentState,
                    robot.isPausedForSafety
                );
                sb.AppendLine(line);
            }
            if (sb.Length > 0) File.AppendAllText(robotLogPath, sb.ToString());
        }
    }
}
