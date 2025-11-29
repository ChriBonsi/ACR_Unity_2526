using System.Collections.Generic;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[System.Serializable]
public class Node
{
    public int id;
    public int x;
    public int y;
    public int type;
    public int[] neighbors;
}

[System.Serializable]
public class MapData
{
    public Node[] nodes;
}

public class MapGenerator : MonoBehaviour
{
    private static ROSConnection ros;
    private MapData mapData;
    public GameObject tilePrefab;
    public GameObject mapParent;
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<StringMsg>("airport_grid/response_airport_grid", ResponseCallback);

        var subscribeMsg = new StringMsg()
        {
            data = "ready"
        };

        ros.Publish("airport_grid/request_airport_grid", subscribeMsg);
    }

    void Update()
    {
        
    }

    private void ResponseCallback(StringMsg msg)
    {
        mapData = JsonUtility.FromJson<MapData>(msg.data);
        SpawnTiles();
    }

    private void SpawnTiles()
    {
        if (tilePrefab == null)
        {
            Debug.LogError("Tile prefab not assigned!");
            return;
        }

        foreach (var node in mapData.nodes)
        {
            Vector3 position = new(node.x, node.y);
            GameObject tile = Instantiate(tilePrefab, position, Quaternion.identity, mapParent.transform);

            Color tileColor = Color.gray;
            tileColor = node.type switch
            {
                0 => Color.red,
                1 => Color.green,
                2 => Color.darkGreen,
                3 => Color.yellow,
                4 => Color.purple,
                _ => Color.gray,
            };
            if (tile.TryGetComponent<Renderer>(out var rend))
            {
                rend.material.color = tileColor;
            }
        }
    } 
}
