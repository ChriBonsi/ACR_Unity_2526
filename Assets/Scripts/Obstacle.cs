using UnityEngine;

public class Obstacle : MonoBehaviour
{
    void Start()
    {
        gameObject.name = gameObject.GetInstanceID().ToString();
    }
}
