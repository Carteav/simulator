﻿using System;
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
        FollowPath
    }

    [SensorType("Control", new[] { typeof(CartPath) })]
    public class CarteavControlSensor : SensorBase
    {
        protected Subscriber<CartPath> Subscribe;
        protected BridgeInstance Bridge;
        protected SimcartInput CartInput;

        private CartState state;
        private CartPath path;
        private int currentPointIndex;
        private Vector3 offset;
        [SerializeField]
        private float speedMps;

        private Transform cartTransform;
        public void Update()
        {
            switch (state)
            {
                case CartState.FollowPath:
                    FollowPathUpdate();
                    break;
            }
            
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            Bridge = bridge;
            Subscribe = OnPathUpdate;
            CartInput = GetComponentInChildren<SimcartInput>();
            state = CartState.Inactive;
            CartInput = transform.parent.GetComponentInChildren<SimcartInput>();
            cartTransform = transform.parent;
            var plugin = bridge.Plugin;

            Ros2BridgeFactory ros2Factory = new Ros2BridgeFactory();
            ros2Factory.RegPublisher<DetectedRadarObjectData, DetectedRadarObjectArray>(plugin,
                Ros2Conversions.ConvertFrom);
            ros2Factory.RegSubscriber<DetectedRadarObjectData, DetectedRadarObjectArray>(plugin,
                Ros2Conversions.ConvertTo);
            ros2Factory.RegSubscriber<CartPath, Carteav.Messages.CartPath>(plugin,
                (bridgeType) => new CartPath(bridgeType));

            bridge.AddSubscriber(Topic, Subscribe);
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Log("Visualize control sensor");
        }

        public override void OnVisualizeToggle(bool state)
        {
            Debug.Log("Visualize toggle control sensor");
        }

        public void OnPathUpdate(CartPath path)
        {
            Debug.Log($"Received path {path.PathId}");
            
            FollowPath(path);
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
            offset = path.Points[0].Point - cartTransform.position;
            CartInput.AccelInput = 20;

        }

        private void FollowPathUpdate()
        {
            //Vector3 originPoint = path.Points[currentPointIndex].Point;
            Vector3 destinationPoint = path.Points[currentPointIndex + 1].Point + offset;
            Vector3 towards = destinationPoint - cartTransform.position;
            if (towards.sqrMagnitude <= float.Epsilon)
            {
                currentPointIndex++;
                if (currentPointIndex >= path.Points.Count)
                {
                    state = CartState.Inactive;
                    return;
                }
                destinationPoint = path.Points[currentPointIndex + 1].Point + offset;
                towards = destinationPoint - cartTransform.position;
            }
            
            Debug.Log($"Towards: {towards}  Normalized: {towards.normalized}  Position: {cartTransform.position}  Index: {currentPointIndex}");
            var towardsNormalized = towards.normalized;
            // if (towardsNormalized != Vector3.zero)
            //     transform.forward = towardsNormalized;
            float angle = Vector3.Angle(cartTransform.forward, towardsNormalized);
            CartInput.SteerInput = angle / 100f;
            //cartTransform.position += towardsNormalized * speedMps * Time.deltaTime;
            Debug.Log($"New Position: {cartTransform.position}");
        }

        public class CartPath
        {
            public CartPath(Carteav.Messages.CartPath path)
            {
                Points = new List<CartPoint>(path.points.ToList().ConvertAll(structPoint => new CartPoint(structPoint)));
                PathId = path.path_id;
                PathLengthM = path.path_length_m;
                PathDurationSec = path.path_duration_sec;
                Cyclic = path.cyclic;
            }

            public List<CartPoint> Points;
            public string PathId;
            public uint PathLengthM;
            public uint PathDurationSec;
            public bool Cyclic;
        }
        
        public class CartPoint
        {
            public CartPoint(Carteav.Messages.CartPoint pointStruct)
            {
                var v = pointStruct.point;
                Point = new Vector3 () { x = (float)v.x, y = (float)v.y, z = (float)v.z };
                MAXVelocityMps = pointStruct.max_velocity_mps;
                ReqVelocityMps = pointStruct.req_velocity_mps;
                CurrentEtaSec = pointStruct.current_eta_sec;
                IsCrosswalk = pointStruct.is_crosswalk;
                IsJunction = pointStruct.is_junction;
                IsSpeedBumpsgoogle = pointStruct.is_speed_bumpsgoogle;
            }
            public Vector3 Point;
            public double MAXVelocityMps;
            public double ReqVelocityMps;
            public Int16 CurrentEtaSec;
            public bool IsCrosswalk;
            public bool IsJunction;
            public bool IsSpeedBumpsgoogle;
        }

       
    }
}