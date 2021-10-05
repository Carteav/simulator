using System;
using System.Collections.Generic;
using Simulator.Bridge;
using Simulator.Sensors;
using Simulator.Sensors.UI;
using UnityEngine;
using System.Linq;
using Carteav.Messages;
using Simulator.Bridge.Data.Ros;
using Simulator.Utilities;
using Time = UnityEngine.Time;
using Vector3 = UnityEngine.Vector3;

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
        protected Publisher<BoundaryCross> BoundaryCrossPublish;
        protected BridgeInstance Bridge;
        protected SimcartInput CartInput;
        
        [SerializeField] private string PathTopic;
        [SerializeField] private string BoundriesTopic;
        [SerializeField] private PolygonCollider2D collider;
        [SerializeField] private Rigidbody2D rigidBody2D;
        private CartState state;
        private CartPath path;
        private int currentPointIndex;

        private Transform cartTransform;
        private float MaxSteering = 0.5f;
        private float MaxAcceleration = 20f;
        private float PointReachRange = 2f;
        private DataHandler handler;
        private SiteBoundaries boundaries;
        private List<CollisionData> collisions = new List<CollisionData>();
        private Rigidbody rigidBody;
        private Vector3 velocity;
        private float startTime;

        

        public void Update()
        {
            var pos = cartTransform.position;
            handler.Update2DPosition(new Vector2(pos.x, pos.z));
        }

        private void Setup()
        {
            
        }
        
        
        protected override void Initialize()
        {
            rigidBody = transform.parent.GetComponentInChildren<Rigidbody>();
            CartInput = transform.parent.GetComponentInChildren<SimcartInput>();
            startTime = Time.time;
            cartTransform = transform.parent.GetChild(0);
            handler = FindObjectOfType<DataHandler>();
            handler.Setup(BoundaryCrossPublish, rigidBody.gameObject);
        }

        protected override void Deinitialize()
        {
            //throw new NotImplementedException();
        }
        

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            Bridge = bridge;
            PathSubscribe = OnPathReceived;
            BoundariesSubscribe = OnBoundariesReceived;

            var plugin = Bridge.Plugin;
            var ros2Factory = plugin.Factory;
            ros2Factory.RegSubscriber<CartPath, CartPathMessage>(plugin, (path) => new CartPath(path));
            ros2Factory.RegSubscriber<SiteBoundaries, SiteBoundriesMessage>(plugin, (boundries) => new SiteBoundaries(boundries));
            ros2Factory.RegPublisher<BoundaryCross, BoundaryCrossMessage>(plugin, Converters.ConvertToBoundaryCross);

            Bridge.AddSubscriber(PathTopic, PathSubscribe);
            Bridge.AddSubscriber(BoundriesTopic, BoundariesSubscribe);
            BoundaryCrossPublish = Bridge.AddPublisher<BoundaryCross>(BoundriesTopic);
            
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Log("Visualize control sensor");
        }

        public override void OnVisualizeToggle(bool state)
        {
            Debug.Log("Visualize toggle control sensor");
            handler?.ToggleBoundaries(state);
        }

        public void OnBoundariesReceived(SiteBoundaries boundaries)
        {
            this.boundaries = boundaries;

            Debug.Log($"Boundaries received {boundaries.MultiPolygons.Count}.");
            if (handler != null)
            {
                handler.HandleBoundaries(boundaries);
            }
        }



        public void OnPathReceived(CartPath path)
        {
            Debug.Log($"Received path {path.PathId}");

            //FollowPath(path);
            if (handler != null)
            {
                Vector3 offset = cartTransform.position - path.Points[0].Point;
                offset.y = 0;
                handler.HandlePath(path, offset);
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


        private void OnCollisionEnter(Collision other)
        {
            Vector3 normal = other.contacts[0].normal;
            collisions.Add(new CollisionData
            {
                ObjectName = other.gameObject.name,
                Position = transform.position,
                Velocity = other.relativeVelocity,
                YawAngle = 90 - (Vector3.Angle(velocity, -normal)),
                Time = Time.time - startTime
            });
        }
                                 
        private void OnTriggerEnter2D(Collider2D other)
        {
            var mapBoundary = other.gameObject.GetComponent<MapBoundary>();
            if (mapBoundary.Type == MapBoundary.BoundaryType.RestrictedArea)
            {
                Debug.Log("restricted area entered");
            }
        }

        private void OnTriggerExit2D(Collider2D other)
        {
            var mapBoundary = other.gameObject.GetComponent<MapBoundary>();  
            if (mapBoundary.Type == MapBoundary.BoundaryType.MainArea) 
            {                                                                
                Debug.Log("main area exited");                        
            }                                                                
        }
        
        /*
            Should be: x=-y`;y=z`; z=x` according to manual:
            https://www.svlsimulator.com/docs/getting-started/conventions/#converting-between-coordinate-systems
        */
        public static Vector3 ConvertCoordinates(Point coords)
        {
            return new Vector3((float)coords.x, (float)coords.z, (float)coords.y);
        }
    }

    
    
    public class CartPath
    {
        public List<CartPoint> Points;
        public string PathId;
        public uint PathLengthM;
        public uint PathDurationSec;
        public bool Cyclic;

        public CartPath(CartPathMessage pathMessage)
        {
            Points = new List<CartPoint>(pathMessage.points.ToList().ConvertAll(structPoint => new CartPoint(structPoint)));
            PathId = pathMessage.path_id;
            PathLengthM = pathMessage.path_length_m;
            PathDurationSec = pathMessage.path_duration_sec;
            Cyclic = pathMessage.cyclic;
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

        public CartPoint(CartPointMessage pointMessageStruct)
        {
            var v = pointMessageStruct.point; 
            //Point = new Vector3() { x = (float)v.x, y = (float)v.z, z = (float)v.y };
            Point = CarteavControlSensor.ConvertCoordinates(v);
            MAXVelocityMps = pointMessageStruct.max_velocity_mps;
            ReqVelocityMps = pointMessageStruct.req_velocity_mps;
            CurrentEtaSec = pointMessageStruct.current_eta_sec;
            IsCrosswalk = pointMessageStruct.is_crosswalk;
            IsJunction = pointMessageStruct.is_junction;
            IsSpeedBumpsgoogle = pointMessageStruct.is_speed_bumpsgoogle;
        }
    }


    public class SiteBoundaries
    {
        public List<SingleSiteBoundry> MultiPolygons;

        public SiteBoundaries(SiteBoundriesMessage boundaries)
        {
            MultiPolygons = boundaries.multi_polygons.ToList()
                .ConvertAll(dataBoundary => new SingleSiteBoundry(dataBoundary));
        }

        public SiteBoundaries() { }
    }

    public class SingleSiteBoundry
    {
        public List<Polygon> Polygons;
        public SingleSiteBoundry(Carteav.Messages.SingleSiteBoundryMessage boundary)
        {
            Polygons = boundary.polygons.ToList().ConvertAll(dataPolygon => new Polygon(dataPolygon));
        }
        public SingleSiteBoundry() { }
    }

    public class Polygon
    {
        public List<Vector3> Points;
        public Vector3 Offset;
        public Polygon(Carteav.Messages.PolygonMessage polygonMessage)
        {
            Points = polygonMessage.points.ToList()
                .ConvertAll(CarteavControlSensor.ConvertCoordinates);
        }

        public Polygon() { }
    }

    public class CollisionData
    {
        public string ObjectName;
        public Vector3 Position;
        public float YawAngle;
        public Vector3 Velocity;
        public float Time;
    }
    
    public class BoundaryCross
    {
        public string ObjectName;
        public Vector3 Position;
        public float YawAngle;
        public Vector3 Velocity;
        public float Time;
        public MapBoundary.BoundaryType BoundaryType;
    }
    
   
}