using System.Collections.Generic;
using Simulator.Bridge;
using Simulator.Sensors;
using Simulator.Sensors.UI;
using UnityEngine;
using Carteav.Messages;
using Simulator.Utilities;
using Vector3 = UnityEngine.Vector3;

namespace Carteav
{
    

    [SensorType("Control", new[] { typeof(CartPath) })]
    public class CarteavControlSensor : SensorBase
    {
        public SiteBoundaries Boundaries => boundaries;
        protected Subscriber<CartPath> PathSubscribe;
        protected Subscriber<SiteBoundaries> BoundariesSubscribe;
        protected Publisher<BoundaryCross> BoundaryCrossPublish;
        protected Publisher<CollisionData> CollisionPublish;
        protected BridgeInstance Bridge;
        protected SimcartInput CartInput;
        
        [SerializeField] private string PathTopic;
        [SerializeField] private string BoundriesTopic;
        [SerializeField] private PolygonCollider2D collider;
        [SerializeField] private Rigidbody2D rigidBody2D;
        
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
        private VehicleController vehicleController;
     

        public void Update()
        {
            var pos = cartTransform.position;
            handler.Update2DPosition(new Vector2(pos.x, pos.z));
        }
        
        
        protected override void Initialize()
        {
            var parent = transform.parent;
            rigidBody = parent.GetComponentInChildren<Rigidbody>();
            CartInput = parent.GetComponentInChildren<SimcartInput>();
            vehicleController = parent.GetComponentInChildren<VehicleController>();
            cartTransform = parent.GetChild(0);
            handler = FindObjectOfType<DataHandler>();
            handler.Setup(BoundaryCrossPublish, rigidBody.gameObject);
            vehicleController.OnCollisionEvent += OnCollision;
        }

        
        protected override void Deinitialize()
        {
            handler.Dispose();
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
            ros2Factory.RegPublisher<BoundaryCross, BoundaryCrossMessage>(plugin, Converters.ConvertBoundaryCross);
            ros2Factory.RegPublisher<CollisionData, CollisionMessage>(plugin, Converters.ConvertCollision);

            Bridge.AddSubscriber(PathTopic, PathSubscribe);
            Bridge.AddSubscriber(BoundriesTopic, BoundariesSubscribe);
            BoundaryCrossPublish = Bridge.AddPublisher<BoundaryCross>(BoundriesTopic);
            CollisionPublish = Bridge.AddPublisher<CollisionData>(BoundriesTopic);
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

        
        private void OnCollision(GameObject obj, GameObject other, Collision collision)
        {
            if (collision == null)
            {
                Debug.Log($"OnCollision: collision null");
                return;
            }
            Vector3 normal = collision.contacts[0].normal;
            var collisionData = new CollisionData
            {
                ObjectName = other.gameObject.name,
                Position = transform.position,
                Velocity = collision.relativeVelocity,
                YawAngle = Vector3.Angle(cartTransform.forward, collision.transform.forward),
                Time = SimulatorManager.Instance.CurrentTime
            };
            Debug.Log($"OnCollision: {Converters.ConvertCollision(collisionData).ToString()}");
            collisions.Add(collisionData);
            CollisionPublish(collisionData);
        }
                                 
      
        public void FollowPath(CartPath path)
        {
            if (path.Points == null || path.Points.Count < 2)
            {
                Debug.LogError($"Path points received are either missing or too few to follow.");
                return;
            }

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
                    //state = CartState.Inactive;
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
}