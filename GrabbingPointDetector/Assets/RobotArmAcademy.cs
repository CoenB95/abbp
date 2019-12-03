using System.Collections;
using System.Collections.Generic;
using MLAgents;
using UnityEngine;

public class RobotArmAcademy : Academy
{
    public override void AcademyReset()
    {
        Physics.gravity = new Vector3(0, -resetParameters["gravity"], 0);
    }

    public override void AcademyStep()
    {
    }
}
