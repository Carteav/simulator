/**
 * Copyright (c) 2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

namespace Simulator.Sensors
{
    using System.Collections.Generic;
    using System.Linq;
    using System.Threading.Tasks;
    using Bridge;
    using Bridge.Data;
    using PointCloud;
    using UI;
    using Unity.Collections.LowLevel.Unsafe;
    using Unity.Profiling;
    using UnityEngine;
    using UnityEngine.Rendering;
    using UnityEngine.Rendering.HighDefinition;
    using UnityEngine.Serialization;
    using Utilities;

    [SensorType("Lidar", new[] { typeof(PointCloudData) })]
    [RequireComponent(typeof(Camera))]
    public class CarteavLidarBase : SensorBase
    {
        protected static readonly Matrix4x4 LidarTransform = new Matrix4x4(new Vector4(0, -1, 0, 0),
            new Vector4(0, 0, 1, 0), new Vector4(1, 0, 0, 0), Vector4.zero);

        protected static class Properties
        {
            public static readonly int InputCubemapTexture = Shader.PropertyToID("_InputCubemapTexture");
            public static readonly int Output = Shader.PropertyToID("_Output");
            public static readonly int LatitudeAngles = Shader.PropertyToID("_LatitudeAngles");
            public static readonly int LaserCount = Shader.PropertyToID("_LaserCount");
            public static readonly int MeasuresPerRotation = Shader.PropertyToID("_MeasurementsPerRotation");
            public static readonly int Origin = Shader.PropertyToID("_Origin");
            public static readonly int Transform = Shader.PropertyToID("_Transform");
            public static readonly int RotMatrix = Shader.PropertyToID("_RotMatrix");
            public static readonly int PackedVec = Shader.PropertyToID("_PackedVec");
            public static readonly int PointCloud = Shader.PropertyToID("_PointCloud");
            public static readonly int LocalToWorld = Shader.PropertyToID("_LocalToWorld");
            public static readonly int Size = Shader.PropertyToID("_Size");
            public static readonly int Color = Shader.PropertyToID("_Color");


            // Custom Shader properties:
            public static readonly int PointsCustom = Shader.PropertyToID("_CustomPoints");
            public static readonly int SectorsCustom = Shader.PropertyToID("_CustomSectors");
            public static readonly int SizeCustom = Shader.PropertyToID("_CustomSize");
            public static readonly int IndexCustom = Shader.PropertyToID("_CustomIndex");
            public static readonly int YawCustom = Shader.PropertyToID("_CustomYaw");
            public static readonly int PitchCustom = Shader.PropertyToID("_CustomPitch");
            public static readonly int ProjectAngles = Shader.PropertyToID("_CustomProjectAngles");
        }

        [HideInInspector] public int TemplateIndex;

        [SensorParameter] public List<float> VerticalRayAngles;

        [SensorParameter] [Range(1, 128)] public int LaserCount = 32;

        [SensorParameter] [Range(1.0f, 45.0f)] public float FieldOfView = 40.0f;

        [SensorParameter] [Range(-45.0f, 45.0f)]
        public float CenterAngle = 10.0f;

        [SensorParameter] [Range(0.01f, 1000f)]
        public float MinDistance = 0.5f; // meters

        [SensorParameter] [Range(0.01f, 2000f)]
        public float MaxDistance = 100.0f; // meters

        [SensorParameter] [Range(1, 30)] public float RotationFrequency = 5.0f; // Hz

        [SensorParameter] [Range(18, 6000)] // minmimum is 360/HorizontalAngleLimit
        public int MeasurementsPerRotation = 1500; // for each ray

        [SensorParameter] public bool Compensated = true;

        [SensorParameter] [Range(1, 10)] public float PointSize = 2.0f;

        [SensorParameter] public Color PointColor = Color.red;

        [Range(0f, Mathf.PI)] public float Angle = Mathf.PI * 0.5f;

        [SensorParameter] public int CubemapSize = 1024;

        public ComputeShader computeShader;

        protected BridgeInstance Bridge;

        protected Publisher<PointCloudData> Publish;

        protected uint Sequence;

        private float NextCaptureTime;

        private Camera sensorCamera;

        private HDAdditionalCameraData hdAdditionalCameraData;

        private readonly int faceMask = 1 << (int)CubemapFace.PositiveX | 1 << (int)CubemapFace.NegativeX |
                                        1 << (int)CubemapFace.PositiveY | 1 << (int)CubemapFace.NegativeY |
                                        1 << (int)CubemapFace.PositiveZ | 1 << (int)CubemapFace.NegativeZ;

        private Camera SensorCamera
        {
            get
            {
                if (sensorCamera == null)
                    sensorCamera = GetComponent<Camera>();

                return sensorCamera;
            }
        }

        private HDAdditionalCameraData HDAdditionalCameraData
        {
            get
            {
                if (hdAdditionalCameraData == null)
                    hdAdditionalCameraData = GetComponent<HDAdditionalCameraData>();

                return hdAdditionalCameraData;
            }
        }

        [FormerlySerializedAs("PerformanceLoad")] [SensorParameter]
        public float performanceLoad = 1.0f;

        public override float PerformanceLoad => performanceLoad;
        public override SensorDistributionType DistributionType => SensorDistributionType.ClientOnly;

        private SensorRenderTarget renderTarget;

        private Material PointCloudMaterial;
        private ShaderTagId passId;
        protected ComputeShader cs;

        protected Vector4[] Points;

        protected ComputeBuffer PointCloudBuffer;
        private ComputeBuffer LatitudeAnglesBuffer;

        private ProfilerMarker RenderMarker = new ProfilerMarker("Lidar.Render");
        private ProfilerMarker ComputeMarker = new ProfilerMarker("Lidar.Compute");
        private ProfilerMarker VisualizeMarker = new ProfilerMarker("Lidar.Visualzie");

        private float MaxAngle;
        private float DeltaLongitudeAngle;
        protected int CurrentLaserCount;
        private int CurrentMeasurementsPerRotation;
        private float CurrentFieldOfView;
        private List<float> CurrentVerticalRayAngles;
        private float CurrentCenterAngle;
        private float CurrentMinDistance;
        private float CurrentMaxDistance;


        // Custom Fields
        protected int TotalPointCount;
        protected virtual int Kernel => cs.FindKernel(Compensated ? "CubeComputeComp" : "CubeCompute");

        protected Vector3Int DispatchThreads;
        //


        protected override void Initialize()
        {
            SensorCamera.enabled = false;
            passId = new ShaderTagId("SimulatorLidarPass");
            cs = Instantiate(computeShader);
            PointCloudMaterial = new Material(RuntimeSettings.Instance.PointCloudShader);
            HDAdditionalCameraData.hasPersistentHistory = true;
            HDAdditionalCameraData.customRender += CustomRender;

            //
            TotalPointCount = LaserCount * MeasurementsPerRotation;
            DispatchThreads = new Vector3Int(HDRPUtilities.GetGroupSize(MeasurementsPerRotation, 8),
                HDRPUtilities.GetGroupSize(LaserCount, 8), 1);
            //

            Reset();
        }

        protected override void Deinitialize()
        {
            renderTarget?.Release();
            renderTarget = null;

            PointCloudBuffer?.Release();
            PointCloudBuffer = null;

            LatitudeAnglesBuffer?.Release();
            LatitudeAnglesBuffer = null;
        }


        public virtual void Reset()
        {
            if (PointCloudBuffer != null)
            {
                PointCloudBuffer.Release();
                PointCloudBuffer = null;
            }

            if (LatitudeAnglesBuffer != null)
            {
                LatitudeAnglesBuffer.Release();
                LatitudeAnglesBuffer = null;
            }

            DeltaLongitudeAngle = 1f / MeasurementsPerRotation;
            MaxAngle = Mathf.Abs(CenterAngle) + FieldOfView / 2.0f;

            float startLatitudeAngle;
            // Assuming center of view frustum is horizontal, find the vertical FOV (of view frustum) that can encompass the tilted Lidar FOV.
            // "MaxAngle" is half of the vertical FOV of view frustum.
            if (VerticalRayAngles.Count == 0)
            {
                MaxAngle = Mathf.Abs(CenterAngle) + FieldOfView / 2.0f;

                startLatitudeAngle = 90.0f + MaxAngle;
                //If the Lidar is tilted up, ignore lower part of the vertical FOV.
                if (CenterAngle < 0.0f)
                {
                    startLatitudeAngle -= MaxAngle * 2.0f - FieldOfView;
                }
            }
            else
            {
                LaserCount = VerticalRayAngles.Count;
                startLatitudeAngle = 90.0f - VerticalRayAngles.Min();
                var endLatitudeAngle = 90.0f - VerticalRayAngles.Max();
                FieldOfView = startLatitudeAngle - endLatitudeAngle;
                MaxAngle = Mathf.Max(startLatitudeAngle - 90.0f, 90.0f - endLatitudeAngle);
                TotalPointCount = LaserCount * MeasurementsPerRotation;
            }

            CurrentVerticalRayAngles = new List<float>(VerticalRayAngles);
            CurrentLaserCount = LaserCount;
            CurrentMeasurementsPerRotation = MeasurementsPerRotation;
            CurrentFieldOfView = FieldOfView;
            CurrentCenterAngle = CenterAngle;
            CurrentMinDistance = MinDistance;
            CurrentMaxDistance = MaxDistance;

            var latitudeAngles = new float[LaserCount];

            if (VerticalRayAngles.Count == 0)
            {
                if (LaserCount == 1)
                {
                    Debug.Log("Center angle: " + CenterAngle);
                    latitudeAngles[0] = 1f - (90f + CenterAngle) * Mathf.Deg2Rad / Mathf.PI;
                }
                else
                {
                    var deltaLatitudeAngle = FieldOfView / LaserCount;
                    var index = 0;
                    var angle = startLatitudeAngle;
                    while (index < LaserCount)
                    {
                        latitudeAngles[index] = 1f - angle * Mathf.Deg2Rad / Mathf.PI;
                        index++;
                        angle -= deltaLatitudeAngle;
                    }
                }
            }
            else
            {
                for (var index = 0; index < LaserCount; index++)
                {
                    latitudeAngles[index] = 1f - ((90.0f - VerticalRayAngles[index]) * Mathf.Deg2Rad / Mathf.PI);
                }
            }

            LatitudeAnglesBuffer = new ComputeBuffer(LaserCount, sizeof(float));
            LatitudeAnglesBuffer.SetData(latitudeAngles);

            var totalCount = TotalPointCount;
            PointCloudBuffer = new ComputeBuffer(totalCount, UnsafeUtility.SizeOf<Vector4>());
            Points = new Vector4[totalCount];

            if (PointCloudMaterial != null)
                PointCloudMaterial.SetBuffer(Properties.PointCloud, PointCloudBuffer);
        }

        protected void CustomRender(ScriptableRenderContext context, HDCamera hd)
        {
            var cmd = CommandBufferPool.Get();

            void RenderPointCloud(CubemapFace face)
            {
                PointCloudManager.RenderLidar(context, cmd, hd, renderTarget.ColorHandle, renderTarget.DepthHandle,
                    face);
            }

            RenderMarker.Begin();
            SensorPassRenderer.Render(context, cmd, hd, renderTarget, passId, Color.clear, RenderPointCloud);
            RenderMarker.End();

            ComputeMarker.Begin();
            int kernel = Kernel;
            if (SetComputeParams(cmd, kernel))
            {
                cmd.DispatchCompute(cs, kernel, DispatchThreads.x, DispatchThreads.y, DispatchThreads.z);
            }
            ComputeMarker.End();

            context.ExecuteCommandBuffer(cmd);
            cmd.Clear();
            CommandBufferPool.Release(cmd);
        }

        protected virtual bool SetComputeParams(CommandBuffer cmd, int kernel)
        {
            cmd.SetComputeTextureParam(cs, kernel, Properties.InputCubemapTexture, renderTarget.ColorHandle);
            cmd.SetComputeBufferParam(cs, kernel, Properties.Output, PointCloudBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.LatitudeAngles, LatitudeAnglesBuffer);
            cmd.SetComputeIntParam(cs, Properties.LaserCount, LaserCount);
            cmd.SetComputeIntParam(cs, Properties.MeasuresPerRotation, MeasurementsPerRotation);
            cmd.SetComputeVectorParam(cs, Properties.Origin, SensorCamera.transform.position);
            cmd.SetComputeMatrixParam(cs, Properties.RotMatrix, Matrix4x4.Rotate(transform.rotation));
            cmd.SetComputeMatrixParam(cs, Properties.Transform, transform.worldToLocalMatrix);
            cmd.SetComputeVectorParam(cs, Properties.PackedVec,
                new Vector4(MaxDistance, DeltaLongitudeAngle, MinDistance, 0f));
            return true;
        }


        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            Bridge = bridge;
            Publish = bridge.AddPublisher<PointCloudData>(Topic);
        }

        public virtual void Update()
        {
            if (LaserCount != CurrentLaserCount ||
                MeasurementsPerRotation != CurrentMeasurementsPerRotation ||
                !Mathf.Approximately(FieldOfView, CurrentFieldOfView) ||
                !Mathf.Approximately(CenterAngle, CurrentCenterAngle) ||
                !Mathf.Approximately(MinDistance, CurrentMinDistance) ||
                !Mathf.Approximately(MaxDistance, CurrentMaxDistance) ||
                !VerticalRayAngles.SequenceEqual(CurrentVerticalRayAngles))
            {
                Reset();
            }

            SensorCamera.fieldOfView = FieldOfView;
            SensorCamera.nearClipPlane = MinDistance;
            SensorCamera.farClipPlane = MaxDistance;

            CheckTexture();
            CheckCapture();
        }

        private void CheckTexture()
        {
            if (renderTarget != null && (!renderTarget.IsCube || !renderTarget.IsValid(CubemapSize, CubemapSize)))
            {
                renderTarget.Release();
                renderTarget = null;
            }

            if (renderTarget == null)
            {
                renderTarget = SensorRenderTarget.CreateCube(CubemapSize, CubemapSize, faceMask);
                SensorCamera.targetTexture = null;
            }
        }

        private void RenderCamera()
        {
            SensorCamera.Render();
        }

        private void CheckCapture()
        {
            if (Time.time >= NextCaptureTime)
            {
                RenderCamera();
                SendMessage();

                if (NextCaptureTime < Time.time - Time.deltaTime)
                {
                    NextCaptureTime = Time.time + 1.0f / RotationFrequency;
                }
                else
                {
                    NextCaptureTime += 1.0f / RotationFrequency;
                }
            }
        }

        protected virtual void SendMessage()
        {
            if (!(Bridge is { Status: Status.Connected }))
                return;

            var worldToLocal = LidarTransform;
            
            if (Compensated)
            {
                worldToLocal = worldToLocal * transform.worldToLocalMatrix;
            }

            PointCloudBuffer.GetData(Points);

            Task.Run(() =>
            {
                Publish(new PointCloudData()
                {
                    Name = Name,
                    Frame = Frame,
                    Time = SimulatorManager.Instance.CurrentTime,
                    Sequence = Sequence++,

                    LaserCount = CurrentLaserCount,
                    Transform = worldToLocal,
                    Points = Points,
                    PointCount = Points.Length
                });
            });
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            VisualizeMarker.Begin();
            var lidarToWorld = Compensated ? Matrix4x4.identity : transform.localToWorldMatrix;
            PointCloudMaterial.SetMatrix(Properties.LocalToWorld, lidarToWorld);
            PointCloudMaterial.SetFloat(Properties.Size, PointSize * Utility.GetDpiScale());
            PointCloudMaterial.SetColor(Properties.Color, PointColor);
            Graphics.DrawProcedural(PointCloudMaterial, new Bounds(transform.position, MaxDistance * Vector3.one),
                MeshTopology.Points, PointCloudBuffer.count, layer: LayerMask.NameToLayer("Sensor"));

            VisualizeMarker.End();
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }
    }
}