using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectSpawner : MonoBehaviour
{
    public GameObject[] objects;
    public int amountPerObject;

    public void SpawnObjects() 
    {
        float maxX = transform.parent.localScale.x;
        float minX = -transform.parent.localScale.x;
        float maxZ = transform.parent.localScale.z;
        float minZ = -transform.parent.localScale.z;

        for (int j = 0; j < amountPerObject; j++) {
            for(int i = 0; i < objects.Length; i++) {
                float x = Random.Range(minX, maxX);
                float z = Random.Range(minZ, maxZ);
                Debug.Log(x + ", " + z);
                GameObject newObject = Instantiate(objects[i], new Vector3(x, transform.position.y, z), Random.rotation);
                newObject.transform.parent = gameObject.transform;
            }
        }
    }
}
