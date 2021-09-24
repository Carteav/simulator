using System.Collections;
using System.Collections.Generic;
using Simulator.Bridge.Data;
using Simulator.Sensors;
using Simulator.Utilities;
using UnityEngine;

namespace Simulator.Sensors
{
    [SensorType("Lidar", new[] { typeof(PointCloudData) })]
    public class RoboBpearlLidar : CarteavLidarSensor
    {
    }
}