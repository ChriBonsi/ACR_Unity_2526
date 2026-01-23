using System.Collections.Generic;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class HumanGenerator : MonoBehaviour
{
    public GameObject humanPrefab;
    public GameObject humanParent;
    private ROSConnection ros;

    private Vector3[] spawn =
    {
        new(2, 1, 33),
        new(2, 1, 18)
    };
    private Vector3[] dest =
    {
        new(50, 1, 33),
        new(50, 1, 18)
    };
    private List<Vector3> pos = new();
    private int maxHuman = 10;
    private static int currentHuman = 0;
    private float spawnTimer = 0.0f;
    private float spawnInterval = 1.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>("airport_grid/response_airport_grid", ResCallback);
    }

    void Update()
    {
        if (pos.Count == 0) return;
        if (spawnTimer <= spawnInterval)
        {
            spawnTimer += Time.deltaTime;
            return;
        }
        if( currentHuman >= maxHuman) return;

        double probGoPos = Random.Range(0f, 1f);
        Vector3 s = spawn[Random.Range(0, spawn.Length)];
        Vector3 d = dest[Random.Range(0, dest.Length)];
        GameObject human = Instantiate(humanPrefab, s, Quaternion.identity, humanParent.transform);
        Human humanScript = human.GetComponent<Human>();
        currentHuman += 1;
        if(probGoPos >= 0.15f)
        {
            List<Vector3> waypoints = new();
            int waypointCount = Random.Range(2, 5);
            for (int i = 0; i < waypointCount; i++)
            {
                waypoints.Add(pos[Random.Range(0, pos.Count)]);
            }
            waypoints.Add(d);
            humanScript.SetDestinations(waypoints);
        }
        else
        {
            humanScript.SetDestinations(new List<Vector3> { d });
        }
        spawnTimer = 0.0f;
    }

    private void ResCallback(StringMsg msg)
    {
        MapData mapData = JsonUtility.FromJson<MapData>(msg.data);
        if (mapData == null || mapData.nodes == null) return;

        foreach (var node in mapData.nodes)
        {
            switch (node.type)
            {
                case 1:
                    pos.Add(new Vector3(node.x, node.y + 1, node.z)); 
                    break;
            }
        }
    }

    public static void DecreaseHumanCount()
    {
        currentHuman -= 1;
    }
}
