using System.Collections.Generic;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class HumanGenerator : MonoBehaviour
{
    public GameObject humanPrefab;
    public GameObject humanParent;
    private ROSConnection ros;

    private Vector3 spawn;
    private Vector3 dest;
    private List<Vector3> pos = new();
    private int maxHuman = 20;
    private static int currentHuman = 0;
    private float spawnTimer = 0.0f;
    private float spawnInterval = 1.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        spawn = new(0, 1, 12);
        dest = new(29, 1, 12);
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
        GameObject human = Instantiate(humanPrefab, spawn, Quaternion.identity, humanParent.transform);
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
            waypoints.Add(dest);
            humanScript.SetDestinations(waypoints);
        }
        else
        {
            humanScript.SetDestinations(new List<Vector3> { dest });
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
