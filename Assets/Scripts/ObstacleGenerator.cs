using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Std;
using System.Collections.Generic;

public class ObstacleGenerator : MonoBehaviour
{
    private class ObstacleNode
    {
        public Vector3 position;
        public float dirt;
        public bool hasObstacle;
        public float spawnReadyTime;
        public bool isUnattended;
    }

    public GameObject obstaclesParent;
    public GameObject dirtPrefab;
    public GameObject unattendedObstaclePrefab;

    private bool ready = false;
    private ROSConnection ros;
    private static Dictionary<string, ObstacleNode> validNodes = new();
    private static Dictionary<string, Vector3> invalidNodes = new();
    private static int currentDirt = 0;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>("airport_grid/response_airport_grid", GridCallback);
    }

    private void Update()
    {
        if (!ready) return;

        foreach (var nodeEntry in validNodes)
        {
            ObstacleNode node = nodeEntry.Value;
            if(node.isUnattended)
            {
                node.isUnattended = false;
                node.hasObstacle = true;
                Instantiate(unattendedObstaclePrefab, node.position + new Vector3(0, 1f, 0), Quaternion.identity, obstaclesParent.transform);
                return;
            }
            if (node.dirt >= 1f && !node.hasObstacle && !RobotNear(node.position))
            {
                if (node.spawnReadyTime == 0f)
                {
                    node.spawnReadyTime = Time.time + 5.0f;
                }
                else if (Time.time >= node.spawnReadyTime)
                {
                    currentDirt += 1;
                    node.hasObstacle = true;
                    node.spawnReadyTime = 0f;
                    Instantiate(dirtPrefab, node.position + new Vector3(0, 1f, 0), Quaternion.identity, obstaclesParent.transform);
                    //Debug.Log($"Obstacle at {node.position} generated dirt. Total dirt: {currentDirt}");
                }
            }
        }
    }

    private bool RobotNear(Vector3 position)
    {
        Collider[] colliderHits = Physics.OverlapSphere(position, 2.0f);
        foreach (var hit in colliderHits)
        {
            if (hit.CompareTag("Robot"))
            {
                return true;
            }
        }
        return false;
    }

    private void GridCallback(StringMsg msg)
    {
        MapData mapData = JsonUtility.FromJson<MapData>(msg.data);
        if (mapData == null || mapData.nodes == null) return;

        foreach (var node in mapData.nodes)
        {
            switch (node.type)
            {
                case 1:
                    invalidNodes.Add($"{node.x},{node.y},{node.z}", new Vector3(node.x, node.y, node.z));
                    break;
                case 6 or 7:
                    validNodes.Add($"{node.x},{node.y},{node.z}", new ObstacleNode
                    {
                        position = new Vector3(node.x, node.y, node.z),
                        dirt = Random.Range(0f, 0.5f),
                    });
                    break;
            }
        }

        ready = true;
    }

    public static void CleanedDirt(string nodeKey)
    {
        ObstacleNode node = validNodes.GetValueOrDefault(nodeKey, null);
        if (node == null) return;
        node.dirt = 0f;
        if(node.hasObstacle) currentDirt = Mathf.Max(0, currentDirt - 1);
        node.hasObstacle = false;
        node.spawnReadyTime = 0f;
    }

    public static void UpdateObstacleDirt(string nodeKey, float amount = 0.0f)
    {
        ObstacleNode node = validNodes.GetValueOrDefault(nodeKey, null);
        if (node == null) return;
        if( amount > 0.0f )
        {
            node.dirt += amount;
        }
        else
        {
            node.dirt += Random.Range(0.01f, 0.2f);
        }        
    }

    private static string GetClosestValidNode(string nodeKey)
    {
        Vector3 nodePos = invalidNodes.GetValueOrDefault(nodeKey, Vector3.zero);
        if (nodePos == Vector3.zero) return null;

        string closestKey = null;
        float closestDistance = float.MaxValue;

        foreach (var validEntry in validNodes)
        {
            float distance = Vector3.Distance(nodePos, validEntry.Value.position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestKey = validEntry.Key;
            }
        }

        return closestKey;
    }

    public static void SpawnUnattendedObstacle(string nodeKey)
    {
        Vector3 node = invalidNodes.GetValueOrDefault(nodeKey, Vector3.zero);
        if (node == Vector3.zero) return;

        string closestValidNodeKey = GetClosestValidNode(nodeKey);
        if (closestValidNodeKey == null) return;

        float chance = Random.Range(0f, 1f);
        ObstacleNode validNode = validNodes[closestValidNodeKey];
        if (chance < 0.3f && !validNode.hasObstacle)
        {
            validNode.isUnattended = true;
        }
    }
}