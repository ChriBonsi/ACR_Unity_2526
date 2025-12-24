using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PathPlanner;
using RosMessageTypes.ObstacleManager;
using RosMessageTypes.RobotManager;
using RosMessageTypes.Std;

public class ROSGlobalPublisher : MonoBehaviour
{
    void Awake()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PathPlannerRequestMsg>("path_planner/request");
        ros.RegisterPublisher<PathPlannerBatteryRequestMsg>("path_planner/battery_request");
        ros.RegisterPublisher<ObstacleManagerReportMsg>("obstacle_manager/report_obstacle");
        ros.RegisterPublisher<StringMsg>("robot_manager/request_robot");
        ros.RegisterPublisher<RobotManagerTrackerMsg>("robot_manager/subscribe_tracker");
        ros.RegisterPublisher<StringMsg>("airport_grid/request_airport_grid");
        ros.RegisterPublisher<StringMsg>("path_planner/reset_state");
        ros.RegisterPublisher<StringMsg>("robot_coordination");
        Debug.Log("Global ROS publishers registered.");

        ros.Publish("path_planner/reset_state", new StringMsg() { data = "reset" });
    }
}
