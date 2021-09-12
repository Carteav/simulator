using System;
using System.Collections;
using System.Collections.Generic;
using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Bridge.Data.Ros;

using Vector3 = UnityEngine.Vector3;

namespace Carteav.Messages
{
    [MessageType("carteav_interfaces/CartPath")]
    public struct CartPath
    {
        public CartPoint[] points;
        public string path_id;
        public uint path_length_m;
        public uint path_duration_sec;
        public bool cyclic;
    }
    
    [MessageType("carteav_interfaces/CartPoint")]
    public struct CartPoint
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
    public struct SiteBoundries
    {
        public SingleSiteBoundry[] multi_polygons;
    }
    
    [MessageType("carteav_interfaces/SingleSiteBoundry")]
    public struct SingleSiteBoundry
    {
        public Polygon[] polygons;
    }

    [MessageType("carteav_interfaces/Polygon")]
    public struct Polygon
    {
        public Point[] points;
    }

    
}


