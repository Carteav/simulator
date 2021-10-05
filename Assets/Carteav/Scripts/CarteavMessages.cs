using System;
using System.Collections;
using System.Collections.Generic;
using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Bridge.Data.Ros;
using Ros = Simulator.Bridge.Data.Ros;


using Vector3 = UnityEngine.Vector3;

namespace Carteav.Messages
{
    [MessageType("carteav_interfaces/CartPath")]
    public struct CartPathMessage
    {
        public CartPointMessage[] points;
        public string path_id;
        public uint path_length_m;
        public uint path_duration_sec;
        public bool cyclic;
    }
    
    [MessageType("carteav_interfaces/CartPoint")]
    public struct CartPointMessage
    {
        public Point point;
        public double max_velocity_mps;
        public double req_velocity_mps;
        public Int16 current_eta_sec;
        public bool is_crosswalk;
        public bool is_junction;
        public bool is_speed_bumpsgoogle;
    }

    
    [MessageType("carteav_interfaces/SiteBoundries")]
    public struct SiteBoundriesMessage
    {
        public SingleSiteBoundryMessage[] multi_polygons;
    }
    
    [MessageType("carteav_interfaces/SingleSiteBoundry")]
    public struct SingleSiteBoundryMessage
    {
        public PolygonMessage[] polygons;
    }

    [MessageType("carteav_interfaces/Polygon")]
    public struct PolygonMessage
    {
        public Point[] points;
    }
    
    [MessageType("carteav_interfaces/BoundaryCross")]
    public struct BoundaryCrossMessage
    {
        public string object_name;
        public Vector3 position;
        public float yaw_angle;
        public Vector3 velocity;
        public float time;
        public string boundary_type;
    }


    public static class Converters
    {
        public static Vector3 ConvertFromVector(Ros.Vector3 v)
        {
            return new Vector3() { x = (float)v.x, y = (float)v.y, z = (float)v.z };
        }
        
        public static Ros.Vector3 ConvertToVector(Vector3 v)
        {
            return new Ros.Vector3() { x = v.x, y = v.y, z = v.z };
        }
        
        public static  Vector3 ConvertFromPoint(Point  v)
        {
            return new Vector3 () { x = (float)v.x, y = (float)v.y, z = (float)v.z };
        }
        
        
        public static Point ConvertToPoint(Vector3 v)
        {
            return new Point() { x = v.x, y = v.y, z = v.z };
        }


        public static Messages.BoundaryCrossMessage ConvertToBoundaryCross(Carteav.BoundaryCross cross)
        {
            return new BoundaryCrossMessage()
            {
                object_name = cross.ObjectName,
                position = cross.Position,
                yaw_angle = cross.YawAngle,
                velocity = cross.Velocity,
                time = cross.Time,
                boundary_type = cross.BoundaryType.ToString()
            };
        }
    }
    
}


