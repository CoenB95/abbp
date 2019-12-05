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

    public override void InitializeAgent()
    {
        academy = FindObjectOfType<Academy>();
    }

    /// <summary>
    /// The agent's four actions correspond to torques on each of the two joints.
    /// </summary>
    public override void AgentAction(float[] vectorAction, string textAction)
    {
        float posX = Mathf.Clamp(vectorAction[0], -1f, 1f);
        float posY = Mathf.Clamp(vectorAction[1], -1f, 1f);
        float posZ = Mathf.Clamp(vectorAction[2], -1f, 1f);
        
        transform.position = new Vector3(posX, posY, posZ);

        float rotationX = Mathf.Clamp(vectorAction[3], -1f, 1f);
        float rotationZ = Mathf.Clamp(vectorAction[4], -1f, 1f);

        transform.rotation = new Quaternion(rotationX, 0, rotationZ, 1f);

        SetReward(10f);
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
}
