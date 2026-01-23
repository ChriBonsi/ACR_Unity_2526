using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PathPlanner;
using System.Collections;

public class Human : MonoBehaviour
{
    private Queue<Vector3> destinationQueue;
    private Queue<Vector3> pathQueue;
    private ROSConnection ros;
    private bool isPathRequestPending = false;
    private string lastNodeKey = "";
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PathPlannerResponseMsg>("path_planner/response", PathResponseCallback);
        pathQueue = new Queue<Vector3>();
        SendHumanPathRequest();
    }

    void Update()
    {
        if (gameObject == null) return;
        Move();
        CheckIfQueuedPointReached();
        if(CheckIfDestinationReached())
        {
            if(destinationQueue.Count == 0)
            {
                //Debug.Log($"[Human {gameObject.GetInstanceID()}] Reached final destination. Destroying human.");
                Destroy(gameObject);
                HumanGenerator.DecreaseHumanCount();
                return;
            }
            StartCoroutine(WaitRoutine());
        }
    }

    private IEnumerator WaitRoutine()
    {
        float waitTime = Random.Range(0.5f, 2.0f);
        yield return new WaitForSeconds(waitTime);
        string currentNode = GetCurrentPositionNode();
        if (ObstacleGenerator.SpawnUnattendedObstacle(currentNode))
        {
            if (SimulationLogger.Instance != null)
            {
                SimulationLogger.Instance.LogEvent("Human", gameObject.GetInstanceID().ToString(), "DroppedUnattendedObstacle", currentNode);
            }
        }
        SendHumanPathRequest();
    }

    private void Move()
    {
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        transform.position =
        Vector3.MoveTowards(transform.position, target, 2f * Time.deltaTime);
        string currentNode = GetCurrentPositionNode();
        if (currentNode != lastNodeKey)
        {
            ObstacleGenerator.UpdateObstacleDirt(currentNode, 0.01f);
            lastNodeKey = currentNode;
        }
    }

    private string GetCurrentPositionNode()
    {
        int x = (int)transform.position.x;
        int y = (int)transform.position.y;
        int z = (int)transform.position.z;
        return $"{x},{y - 1},{z}";
    }

    private void CheckIfQueuedPointReached()
    {
        if (pathQueue.Count == 0) return;
        Vector3 target = pathQueue.Peek();
        if (Vector3.Distance(transform.position, target) < 0.02f)
        {
            pathQueue.Dequeue();
        }
    }

    private bool CheckIfDestinationReached()
    {
        if(destinationQueue.Count == 0) return true;
        Vector3 dest = destinationQueue.Peek();
        if (Vector3.Distance(transform.position, dest) < 0.1f)
        {
            destinationQueue.Dequeue();
            return true;
        }
        return false;
    }

    private void SendHumanPathRequest()
    {
        if (destinationQueue == null || destinationQueue.Count == 0 || isPathRequestPending) return;

        Vector3 start = transform.position;
        Vector3 end = destinationQueue.Peek();
        
        var req = new PathPlannerRequestMsg()
        {
            robot_id = gameObject.GetInstanceID(),
            start_x = start.x,
            start_y = start.y,
            start_z = start.z,
            end_x = end.x,
            end_y = end.y,
            end_z = end.z
        };

        //Debug.Log($"[Human {gameObject.GetInstanceID()}] Sending path request from ({start.x}, {start.y}, {start.z}) to ({end.x}, {end.y}, {end.z})");
        ros.Publish("path_planner/request", req);
        isPathRequestPending = true;
    }

    private void PathResponseCallback(PathPlannerResponseMsg msg)
    {
        if (this == null) return;
        if (msg.robot_id != gameObject.GetInstanceID()) return;

        if(!msg.success)
        {
            //Debug.LogWarning($"[Human {gameObject.GetInstanceID()}] Path planning failed.");
            isPathRequestPending = false;
            return;
        }

        //Debug.Log($"[Human {gameObject.GetInstanceID()}] Received path with {msg.path_x.Length} points.");
        pathQueue.Clear();
        for (int i = 0; i < msg.path_x.Length; i++)
        {
            var point = new Vector3
            {
                x = msg.path_x[i],
                y = msg.path_y[i],
                z = msg.path_z[i]
            };
            pathQueue.Enqueue(point);
        }

        if(msg.path_x[^1] != destinationQueue.Peek().x ||
           msg.path_y[^1] != destinationQueue.Peek().y ||
           msg.path_z[^1] != destinationQueue.Peek().z)
        {
            //Debug.LogWarning($"[Human {gameObject.GetInstanceID()}] Final path point does not match destination.");
            pathQueue.Enqueue(destinationQueue.Peek());
        }
        isPathRequestPending = false;
    }

    public void SetDestinations(List<Vector3> destinations)
    {
        destinationQueue = new Queue<Vector3>(destinations);
    }
}
