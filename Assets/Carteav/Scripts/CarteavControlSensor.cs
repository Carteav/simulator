using System;
using System.Collections;
using System.Collections.Generic;
using Carteav;
using Carteav.Messages;
using OsmSharp.API;
using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Bridge.Ros2;
using Simulator.Bridge.Ros2.Lgsvl;
using Simulator.Sensors;
using Simulator.Sensors.UI;
using UnityEngine;
using System.Linq;
using Simulator.Utilities;

namespace Carteav
{
    public enum CartState
    {
        Inactive,
        FollowPath,
        BoundaryCheck
    }

    [SensorType("Control", new[] { typeof(CartPath) })]
    public class CarteavControlSensor : SensorBase
    {
        protected Subscriber<CartPath> PathSubscribe;
        protected Subscriber<SiteBoundaries> BoundariesSubscribe;
        protected BridgeInstance Bridge;
        protected SimcartInput CartInput;

        [SerializeField] private string PathTopic;
        [SerializeField] private string BoundriesTopic;
      

        private CartState state;
        private CartPath path;
        private int currentPointIndex;

        private Transform cartTransform;
        private float MaxSteering = 0.5f;
        private float MaxAcceleration = 20f;
        private float PointReachRange = 2f;
        private DataVisualizer visualizer;
        private SiteBoundaries boundaries;

        public void Update()
        {
            switch (state)
            {
                case CartState.BoundaryCheck:
                    CheckInBoundaries();
                    break;
                case CartState.FollowPath:
                    FollowPathTick();
                    break;
            }
        }
        
        private bool CheckInBoundaries()
        {
            Polygon mainBoundryPolygon = boundaries.MultiPolygons[0].Polygons[0];
            Vector3 position = transform.position;
            if (position.IsInPolygon(mainBoundryPolygon.Points))
            {
                for (int i = 1; i < boundaries.MultiPolygons[0].Polygons.Count; i++)
                {
                    Polygon polygon = boundaries.MultiPolygons[0].Polygons[i];
                    if (position.IsInPolygon(polygon.Points))
                    {
                        // Inside one of the holes in the polygon - non-permitted area
                        Debug.Log($"Cart inside non-permitted hole area at point {position}");
                        return false;
                    }
                }
            }
            else
            {
                Debug.Log($"Cart left main boundary area at point {position}");
                return false;
            }

            return true;
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            Bridge = bridge;
            PathSubscribe = OnPathReceived;
            BoundariesSubscribe = OnBoundariesReceived;
            CartInput = GetComponentInChildren<SimcartInput>();
            state = CartState.Inactive;
            CartInput = transform.parent.GetComponentInChildren<SimcartInput>();
            cartTransform = transform.parent;
            visualizer = FindObjectOfType<DataVisualizer>();

            var plugin = bridge.Plugin;

            Ros2BridgeFactory ros2Factory = new Ros2BridgeFactory();
            ros2Factory.RegSubscriber<CartPath, Carteav.Messages.CartPath>(plugin,
                (path) => new CartPath(path));
            ros2Factory.RegSubscriber<SiteBoundaries, Carteav.Messages.SiteBoundries>(plugin,
                (boundries) => new SiteBoundaries(boundries));

            bridge.AddSubscriber(PathTopic, PathSubscribe);
            bridge.AddSubscriber(BoundriesTopic, BoundariesSubscribe);
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Log("Visualize control sensor");
        }

        public override void OnVisualizeToggle(bool state)
        {
            Debug.Log("Visualize toggle control sensor");
        }

        public void OnBoundariesReceived(SiteBoundaries boundaries)
        {
            this.boundaries = boundaries;
            Debug.Log($"Boundaries received {boundaries.MultiPolygons.Count}.");
            if (visualizer != null)
            {
                visualizer.VisualizeBoundaries(boundaries, 100000f);
            }
        }


        public void OnPathReceived(CartPath path)
        {
            Debug.Log($"Received path {path.PathId}");

            //FollowPath(path);
            if (visualizer != null)
            {
                Vector3 offset = cartTransform.position - path.Points[0].Point;
                offset.y = 0;
                visualizer.VisualizePath(path, offset);
            }
        }


        public void FollowPath(CartPath path)
        {
            if (path.Points == null || path.Points.Count < 2)
            {
                Debug.LogError($"Path points received are either missing or too few to follow.");
                return;
            }

            state = CartState.FollowPath;
            currentPointIndex = 0;
            this.path = path;
        }


        

        private void FollowPathTick()
        {
            Vector3 offset = path.Points[0].Point - cartTransform.position;
            offset.y = 0;
            //Vector3 originPoint = path.Points[currentPointIndex].Point;
            Vector3 destinationPoint = path.Points[currentPointIndex + 1].Point + offset;
            Vector3 cartPosition = cartTransform.position;
            destinationPoint.y = 0;
            cartPosition.y = 0;
            Vector3 towards = destinationPoint - cartPosition;

            if (towards.sqrMagnitude <= PointReachRange)
            {
                currentPointIndex++;
                if (currentPointIndex >= path.Points.Count)
                {
                    state = CartState.Inactive;
                    return;
                }

                destinationPoint = path.Points[currentPointIndex + 1].Point + offset;
                towards = destinationPoint - cartPosition;
            }

            var towardsNormalized = towards.normalized;
            var cartForwards = cartTransform.forward;
            float angle = Vector3.SignedAngle(cartForwards, towardsNormalized, cartTransform.up);
            float distance = Vector3.Distance(cartPosition, destinationPoint);
            var steering = Mathf.Clamp(angle / 180f, -MaxSteering, MaxSteering);
            var acceleration = (PointReachRange < distance ? 1 : distance / PointReachRange) *
                               (MaxSteering - Mathf.Abs(steering)) * MaxAcceleration;
            CartInput.AccelInput = acceleration;
            CartInput.SteerInput = steering;
            Debug.Log($"Position:{cartPosition}  Destination:{destinationPoint}  Angle:{angle}  " +
                      $"Steering:{steering}  Acceleration:{acceleration}\n" +
                      $"Distance:{distance}  " +
                      $"Index:{currentPointIndex}  Towards:{towards}  Normalized:{towardsNormalized}");
        }
    }


    public class CartPath
    {
        public List<CartPoint> Points;
        public string PathId;
        public uint PathLengthM;
        public uint PathDurationSec;
        public bool Cyclic;

        public CartPath(Carteav.Messages.CartPath path)
        {
            Points = new List<CartPoint>(path.points.ToList().ConvertAll(structPoint => new CartPoint(structPoint)));
            PathId = path.path_id;
            PathLengthM = path.path_length_m;
            PathDurationSec = path.path_duration_sec;
            Cyclic = path.cyclic;
        }
    }

    public class CartPoint
    {
        public Vector3 Point;
        public double MAXVelocityMps;
        public double ReqVelocityMps;
        public Int16 CurrentEtaSec;
        public bool IsCrosswalk;
        public bool IsJunction;
        public bool IsSpeedBumpsgoogle;

        public CartPoint(Carteav.Messages.CartPoint pointStruct)
        {
            var v = pointStruct.point;
            Point = new Vector3() { x = (float)v.x, y = (float)v.z, z = (float)v.y };
            MAXVelocityMps = pointStruct.max_velocity_mps;
            ReqVelocityMps = pointStruct.req_velocity_mps;
            CurrentEtaSec = pointStruct.current_eta_sec;
            IsCrosswalk = pointStruct.is_crosswalk;
            IsJunction = pointStruct.is_junction;
            IsSpeedBumpsgoogle = pointStruct.is_speed_bumpsgoogle;
        }
    }


    public class SiteBoundaries
    {
        public List<SingleSiteBoundry> MultiPolygons;

        public SiteBoundaries(Carteav.Messages.SiteBoundries boundaries)
        {
            MultiPolygons = boundaries.multi_polygons.ToList()
                .ConvertAll(dataBoundary => new SingleSiteBoundry(dataBoundary));
        }

        public SiteBoundaries() { }
    }

    public class SingleSiteBoundry
    {
        public List<Polygon> Polygons;
        public SingleSiteBoundry(Carteav.Messages.SingleSiteBoundry boundary)
        {
            Polygons = boundary.polygons.ToList().ConvertAll(dataPolygon => new Polygon(dataPolygon));
        }
        public SingleSiteBoundry() { }
    }

    public class Polygon
    {
        public List<Vector3> Points;
        public Polygon(Carteav.Messages.Polygon polygon)
        {
            Points = polygon.points.ToList()
                .ConvertAll(v => new Vector3() { x = (float)v.x, y = (float)v.y, z = (float)v.z });
        }

        public Polygon() { }
    }

    public static class Extensions
    {
        public static bool IsInPolygon(this Vector3 testPoint, IList<Vector3> vertices)
        {
            if (vertices.Count < 3) return false;
            bool isInPolygon = false;
            var lastVertex = vertices[vertices.Count - 1];
            foreach (var vertex in vertices)
            {
                if (testPoint.y.IsBetween(lastVertex.y, vertex.y))
                {
                    double t = (testPoint.y - lastVertex.y) / (vertex.y - lastVertex.y);
                    double x = t * (vertex.x - lastVertex.x) + lastVertex.x;
                    if (x >= testPoint.x) isInPolygon = !isInPolygon;
                }
                else
                {
                    if (testPoint.y == lastVertex.y && testPoint.x < lastVertex.x && vertex.y > testPoint.y)
                        isInPolygon = !isInPolygon;
                    if (testPoint.y == vertex.y && testPoint.x < vertex.x && lastVertex.y > testPoint.y)
                        isInPolygon = !isInPolygon;
                }

                lastVertex = vertex;
            }

            return isInPolygon;
        }

        public static bool IsBetween(this float x, float a, float b)
        {
            return (x - a) * (x - b) < 0;
        }
    }
}