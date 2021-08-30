using System.Collections;
using System.Collections.Generic;
using Simulator.Sensors;
using UnityEngine;

namespace Simulator.Sensors
{
    public class RoboBpearlLidar : CarteavLidarSensor
    {
        public override void Init()
        {
            centerAngleModifier = 2;
            base.Init();
        }
    }
}