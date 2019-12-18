using System.Collections;
using System.Collections.Generic;
using MLAgents;
using UnityEngine;

public class RobotArmAgent : Agent
{
    private Academy academy;
    public Camera renderCamera;
    public RobotArea area;
    public bool maskActions = true;

    /// <summary>
    /// Amount of rays that come out of the robotarm
    /// </summary>
    int rayAmount = 8;

    /// <summary>
    /// Radius of ray in the outside circle
    /// </summary>
    float radius1 = 0.0075f;
    /// <summary>
    /// Radius of ray in the inside circle
    /// </summary>
    float radius2 = 0.015f;
    /// <summary>
    /// The max distance that the raycasts go
    /// </summary>
    float distance = 0.05f;

    float[] distanceList;



    /// <summary>
    /// Update is called every frame, if the MonoBehaviour is enabled.
    /// </summary>
    void Update()
    {
        this.distanceList = suckDistance();
    }

    public override void InitializeAgent()
    {
        academy = FindObjectOfType<Academy>();
    }

    /// <summary>
    /// The agent's four actions correspond to torques on each of the two joints.
    /// </summary>
    public override void AgentAction(float[] vectorAction)
    {
        float posX = Mathf.Clamp(vectorAction[0], -1f, 1f);
        float posY = Mathf.Clamp(vectorAction[1], -1f, 1f);
        float posZ = Mathf.Clamp(vectorAction[2], -1f, 1f);
        
        transform.position = new Vector3(posX, posY, posZ);

        float rotationX = Mathf.Clamp(vectorAction[3], -1f, 1f);
        float rotationZ = Mathf.Clamp(vectorAction[4], -1f, 1f);

        transform.rotation = new Quaternion(rotationX, 0, rotationZ, 1f);

        var score = suckScore(this.distanceList);

        SetReward(score);
        Done();
    }

    public override void CollectObservations()
    {
        // There are no numeric observations to collect as this environment uses visual
        // observations.
    }

    public override float[] Heuristic()
    {
        var action = new float[5];
        action[0] = Input.GetAxis("Horizontal");
        action[2] = Input.GetAxis("Vertical");

        if (Input.GetKey(KeyCode.LeftShift)) {
            action[1] = transform.position.y - 0.1f;
        }
        if (Input.GetKey(KeyCode.Space)) {
            action[1] = transform.position.y + 0.1f;
        }
        if (Input.GetKey(KeyCode.E)) {
            action[3] = transform.rotation.x + 0.1f;
        }
        if (Input.GetKey(KeyCode.Q)) {
            action[3] = transform.rotation.x - 0.1f; 
        }
        if (Input.GetKey(KeyCode.X)) {
            action[4] = transform.rotation.z - 0.1f; 
        }
        if (Input.GetKey(KeyCode.Z)) {
            action[4] = transform.rotation.z + 0.1f; 
        }
        
        return action;
    }

    // public override void InitializeAgent()
    // {
    //     gameObject.transform.position = new Vector3(2f, 4f, 0f);
    //     gameObject.transform.rotation = new Quaternion(0f, 0f, 0f, 0f);
    //     spawner.SpawnObjects();
    // }

    public override void AgentReset()
    {
        Debug.Log("Wollah reset");
        transform.position = new Vector3(2f, 4f, 0f);
        transform.rotation = new Quaternion(0f, 0f, 0f, 0f);
        area.ResetArea();
    }

    float[] suckDistance()
    {
        float[] distList = new float[rayAmount * 2 + 1];
        float x;
        float z;
        Vector3 xVec;
        Vector3 zVec;
        RaycastHit hitInfo;

        Ray ray = new Ray(transform.position, -transform.up);
        Debug.DrawLine(ray.origin, ray.origin + ray.direction * distance, Color.green);
        
        Physics.Raycast(ray, out hitInfo, distance);

        if (hitInfo.distance == 0) 
            distList[0] = 0;
        else    
            distList[0] = distance - hitInfo.distance; ;

        for (int i = 1; i < rayAmount + 1; i++)
        {
            x = radius1 * Mathf.Sin((i * (360 / rayAmount)) * (Mathf.PI / 180));
            z = radius1 * Mathf.Cos((i * (360 / rayAmount)) * (Mathf.PI / 180));
            xVec = new Vector3(x, x, x);
            zVec = new Vector3(z, z, z);

            ray = new Ray(transform.position + Vector3.Scale(transform.right, xVec) + Vector3.Scale(transform.forward, zVec), -transform.up);
            Debug.DrawLine(ray.origin, ray.origin + ray.direction * distance, Color.green);

            Physics.Raycast(ray, out hitInfo, distance);
            if (hitInfo.distance == 0)
                distList[i] = 0;
            else
                distList[i] = distance - hitInfo.distance;
        }

        for (int i = 1; i < rayAmount + 1; i++)
        {
            x = radius2 * Mathf.Sin((i * (360 / rayAmount)) * (Mathf.PI / 180));
            z = radius2 * Mathf.Cos((i * (360 / rayAmount)) * (Mathf.PI / 180));
            xVec = new Vector3(x, x, x);
            zVec = new Vector3(z, z, z);

            ray = new Ray(transform.position + Vector3.Scale(transform.right, xVec) + Vector3.Scale(transform.forward, zVec), -transform.up);
            Debug.DrawLine(ray.origin, ray.origin + ray.direction * distance, Color.green);

            Physics.Raycast(ray, out hitInfo, distance);
            if (hitInfo.distance == 0)
                distList[rayAmount + 1] = 0;
            else
                distList[rayAmount + i] = distance - hitInfo.distance;
        }

        return distList;
    }

    int suckScore(float[] distList)
    {
        int scaling = 1000;
        int somValue = 0;
        
        for(int i = 0; i < distList.Length; i++)
        {
            somValue += Mathf.RoundToInt(distList[i] *  scaling);
        }

        return somValue;
    }
}
