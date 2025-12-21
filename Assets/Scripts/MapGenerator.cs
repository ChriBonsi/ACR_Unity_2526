using System.Collections.Generic;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UIElements;

[System.Serializable]
public class Node
{
    public int id;
    public int x;
    public int y;
    public int z;
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
            Vector3 position = new(node.x, node.y, node.z);
            GameObject tile = Instantiate(tilePrefab, position, Quaternion.identity, mapParent.transform);

            tile.name = $"Tile_{node.id}_({node.x},{node.y},{node.z})";

            Color tileColor = Color.gray;
            tileColor = node.type switch
            {
                0 => Color.black,
                1 => Color.red,
                2 => Color.brown,
                22 => Color.burlywood,
                3 => Color.skyBlue,
                33 => Color.cyan,
                4 => Color.gray,
                44 => Color.gray,
                5 => Color.darkGray,
                55 => Color.darkGray,
                6 => Color.lightGreen,
                7 => Color.green,
                9 => Color.yellow,
                _ => Color.pink,
            };
            
            if (!tile.TryGetComponent<BoxCollider>(out var collider))
            {
                collider = tile.AddComponent<BoxCollider>();
            }

            switch (node.type)
            {
                case 0 or 1 or 22 or 33 or 44 or 55:
                    collider.isTrigger = true;
                    tile.tag = "Invalid";

                    for (int i = 1; i <= 2; i++)
                    {
                        Vector3 blockPos = new(node.x, i, node.z);
                        GameObject block = Instantiate(tilePrefab, blockPos, Quaternion.identity, mapParent.transform);
                        block.name = $"Building_{node.id}_Level_{i}";
                        
                        if (block.TryGetComponent<Renderer>(out var blockRend))
                        {
                            blockRend.material.color = Color.white;
                        }
                    }
                    break;
                default:
                    collider.isTrigger = false;
                    break;
            }

            if (tile.TryGetComponent<Renderer>(out var rend))
            {
                rend.material.color = tileColor;
            }
        }
    } 
}
