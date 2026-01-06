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
    }

    public GameObject obstaclesParent;
    public GameObject dirtPrefab;

    private bool ready = false;
    private ROSConnection ros;
    private static Dictionary<string, ObstacleNode> validNodes = new();
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
            if (node.dirt >= 1f && !node.hasObstacle)
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

    private void GridCallback(StringMsg msg)
    {
        MapData mapData = JsonUtility.FromJson<MapData>(msg.data);
        if (mapData == null || mapData.nodes == null) return;

        foreach (var node in mapData.nodes)
        {
            switch (node.type)
            {
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

    public static void UpdateObstacleDirt(string nodeKey)
    {
        ObstacleNode node = validNodes.GetValueOrDefault(nodeKey, null);
        if (node == null) return;

        node.dirt += Random.Range(0.01f, 0.2f);
    }
}