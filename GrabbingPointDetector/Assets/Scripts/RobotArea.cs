using System.Collections;
using System.Collections.Generic;
using MLAgents;
using UnityEngine;

public class RobotArea : MonoBehaviour
{
    public ObjectSpawner spawner;

    // Start is called before the first frame update
    void Start()
    {
        spawner.SpawnObjects();
    }

    public void ResetArea()
    {
        foreach (Transform child in transform) {
            if (child.CompareTag("Object")) {
                Destroy(child);
            }
        }
    }
}
