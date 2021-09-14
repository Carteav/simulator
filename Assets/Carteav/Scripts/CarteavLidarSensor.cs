﻿// Decompiled with JetBrains decompiler
// Type: Simulator.Sensors.LidarSensor
// Assembly: LidarSensor, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null
// MVID: 5B25C6C6-76B2-4012-AFA7-4D497FAB39B7
// Assembly location: /home/me/.config/unity3d/LGElectronics/SVLSimulator/Sensors/aa5e3262-fde7-4445-b86e-0922ed5fedba (1)/LidarSensor.dll

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Utilities;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.Serialization;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

namespace Simulator.Sensors
{
    /// <summary>
    /// CarteavLidarSensor is a custom lidar sensor.
    /// Instance fields are named with a "custom" prefix to differentiate from default implementation variables.
    /// </summary>
    [SensorType("Lidar", new[] { typeof(PointCloudData) })]
    //[RequireComponent(typeof (Camera))]
    public class CarteavLidarSensor : CarteavLidarBase
    {
        /* Custom Properties */
        public class PassData
        {
            public List<List<float>> P1;
            public List<List<float>> P2;
            public int[] Lines;
            public float[,] CustomYaw;
            public float[,] CustomPitch;
            public int[] CustomSize;
            public int[] CustomIndex;
            public ComputeBuffer CustomSizeBuffer;
            public ComputeBuffer CustomIndexBuffer;
            public ComputeBuffer CustomYawBuffer;
            public ComputeBuffer CustomPitchBuffer;
            public int MaxPointsPerSector;
        }
        
        [SensorParameter] public bool PointCloudTransform = true;
        [SerializeField] private bool Custom;
        //[SerializeField] private ComputeShader customLidarComputeShader;
        [SerializeField] private string anglesFile;
        [Range(1, 360)] [SerializeField] private int anglePassAmount;
        [SerializeField] private int anglesPerPass;
        [SerializeField] private float timePerPass;
        
        
        
            
        protected BaseLink baseLink;
        protected Dictionary<int, PassData> filePassData = new Dictionary<int, PassData>();
        
        private int customPoints = 100000;
        private int customSectors = 24;
        private bool lastFrameCustom;
        private bool indexReset = false;
        private int currentIndexOffset = 0;
        private float timeSincePass;
        private int passIndex = 0;
        private float minAngle = float.MaxValue;
        private float maxAngle = float.MinValue;
        //
        protected override int Kernel => computeShader.FindKernel(Compensated ? "CarteavCubeComputeComp" : "CarteavCubeCompute");

        public override void Reset()
        {
            TotalPointCount = customPoints;
            if (!Custom)
            {
                DispatchThreads = new Vector3Int(HDRPUtilities.GetGroupSize(MeasurementsPerRotation, 8),
                    HDRPUtilities.GetGroupSize(LaserCount, 8), 1);
            }
            
            base.Reset();
        }

        /*protected override void EndReadRequest(CommandBuffer cmd, ReadRequest req)
        {
            //UnityEngine.Debug.DebugBreak();
            EndReadMarker.Begin();
            string kernelName = Custom ? Compensated != null ? "LidarCustomComputeComp" : "LidarCustomCompute" :
                Compensated != null ? "LidarComputeComp" : "LidarCompute";

            int kernel = cs.FindKernel(kernelName);
            cmd.SetComputeTextureParam(cs, kernel, Properties.Input, req.TextureSet.ColorTexture);
            cmd.SetComputeBufferParam(cs, kernel, Properties.Output, PointCloudBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.SinLatitudeAngles, SinLatitudeAnglesBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.CosLatitudeAngles, CosLatitudeAnglesBuffer);
            cmd.SetComputeIntParam(cs, Properties.Index, req.Index);
            cmd.SetComputeIntParam(cs, Properties.Count, req.Count);
            cmd.SetComputeIntParam(cs, Properties.LaserCount, CurrentLaserCount);
            cmd.SetComputeIntParam(cs, Properties.MeasuresPerRotation, CurrentMeasurementsPerRotation);
            cmd.SetComputeVectorParam(cs, Properties.Origin, req.Origin);
            cmd.SetComputeMatrixParam(cs, Properties.Transform, req.Transform);
            cmd.SetComputeMatrixParam(cs, Properties.CameraToWorld, req.CameraToWorldMatrix);
            cmd.SetComputeVectorParam(cs, Properties.ScaleDistance,
                new Vector4(XScale, YScale, MinDistance, MaxDistance));
            cmd.SetComputeVectorParam(cs, Properties.TexSize,
                new Vector4(RenderTextureWidth, RenderTextureHeight, 1f / RenderTextureWidth,
                    1f / RenderTextureHeight));
            cmd.SetComputeVectorParam(cs, Properties.LongitudeAngles,
                new Vector4(SinStartLongitudeAngle, CosStartLongitudeAngle, DeltaLongitudeAngle, 0.0f));

            if (Custom)
            {
                CustomDispatch(cmd, req, kernel);
            }
            else
            {
                cmd.DispatchCompute(cs, kernel, HDRPUtilities.GetGroupSize(req.Count, 8),
                    HDRPUtilities.GetGroupSize(LaserCount, 8), 1);
            }
            
            EndReadMarker.End();
        }*/

        
        /*
        protected override void SendMessage()
        {
            if (Bridge == null || Bridge.Status != Status.Connected)
                return;
            Matrix4x4 pointsTransform = LidarTransform;
            Matrix4x4 worldToLocal;
            if (PointCloudTransform)
            {
                worldToLocal = baseLink.transform.worldToLocalMatrix;
            }
            else
            {
                worldToLocal = transform.worldToLocalMatrix;
            }

            if (Compensated)
            {
                pointsTransform = pointsTransform * worldToLocal;
            }
                
            
            PointCloudBuffer.GetData(Points);
            Task.Run(() =>
            {
                // ISSUE: variable of the null type
                var publish = Publish;
                PointCloudData pointCloudData = new PointCloudData();
                pointCloudData.Name = Name;
                pointCloudData.Frame = Frame;
                pointCloudData.Time = SimulatorManager.Instance.CurrentTime;
                uint sendSequence = SendSequence;
                SendSequence = sendSequence + 1;
                pointCloudData.Sequence = sendSequence;
                pointCloudData.LaserCount = CurrentLaserCount;
                pointCloudData.Transform = pointsTransform;
                pointCloudData.Points = Points;
                publish.Invoke(pointCloudData);
            });
        }
        */

        /*private void OnValidate()
        {
            if (TemplateIndex == null)
                return;
            Template template = Template.Templates[TemplateIndex];
            if (LaserCount == template.LaserCount && MinDistance == template.MinDistance &&
                (MaxDistance == template.MaxDistance && RotationFrequency == template.RotationFrequency) &&
                (MeasurementsPerRotation == template.MeasurementsPerRotation && FieldOfView == template.FieldOfView &&
                 (CenterAngle == template.CenterAngle && VerticalRayAngles.SequenceEqual(template.VerticalRayAngles))))
                return;
            TemplateIndex = 0;
        }*/

        /*public struct Template
        {
            public string Name;
            public int LaserCount;
            public float MinDistance;
            public float MaxDistance;
            public float RotationFrequency;
            public int MeasurementsPerRotation;
            public float FieldOfView;
            public List<float> VerticalRayAngles;
            public float CenterAngle;

            public static readonly Template[] Templates = new Template[7]
            {
                new Template
                {
                    Name = "Custom",
                    LaserCount = 32,
                    MinDistance = 0.5f,
                    MaxDistance = 100f,
                    RotationFrequency = 10f,
                    MeasurementsPerRotation = 360,
                    FieldOfView = 41.33f,
                    VerticalRayAngles = new List<float>(),
                    CenterAngle = 10f
                },
                new Template
                {
                    Name = "Lidar16",
                    LaserCount = 16,
                    MinDistance = 0.5f,
                    MaxDistance = 100f,
                    RotationFrequency = 10f,
                    MeasurementsPerRotation = 1000,
                    FieldOfView = 30f,
                    VerticalRayAngles = new List<float>(),
                    CenterAngle = 0.0f
                },
                new Template
                {
                    Name = "Lidar16b",
                    LaserCount = 16,
                    MinDistance = 0.5f,
                    MaxDistance = 100f,
                    RotationFrequency = 10f,
                    MeasurementsPerRotation = 1500,
                    FieldOfView = 20f,
                    VerticalRayAngles = new List<float>(),
                    CenterAngle = 0.0f
                },
                new Template
                {
                    Name = "Lidar32",
                    LaserCount = 32,
                    MinDistance = 0.5f,
                    MaxDistance = 100f,
                    RotationFrequency = 10f,
                    MeasurementsPerRotation = 1500,
                    FieldOfView = 41.33f,
                    VerticalRayAngles = new List<float>(),
                    CenterAngle = 10f
                },
                new Template
                {
                    Name = "Lidar32-NonUniform",
                    LaserCount = 32,
                    MinDistance = 0.5f,
                    MaxDistance = 100f,
                    RotationFrequency = 10f,
                    MeasurementsPerRotation = 1500,
                    FieldOfView = 41.33f,
                    VerticalRayAngles = new List<float>
                    {
                        -25f,
                        -1f,
                        -1.667f,
                        -15.639f,
                        -11.31f,
                        0.0f,
                        -0.667f,
                        -8.843f,
                        -7.254f,
                        0.333f,
                        -0.333f,
                        -6.148f,
                        -5.333f,
                        1.333f,
                        0.667f,
                        -4f,
                        -4.667f,
                        1.667f,
                        1f,
                        -3.667f,
                        -3.333f,
                        3.333f,
                        2.333f,
                        -2.667f,
                        -3f,
                        7f,
                        4.667f,
                        -2.333f,
                        -2f,
                        15f,
                        10.333f,
                        -1.333f
                    },
                    CenterAngle = 10f
                },
                new Template
                {
                    Name = "Lidar64",
                    LaserCount = 64,
                    MinDistance = 0.5f,
                    MaxDistance = 120f,
                    RotationFrequency = 5f,
                    MeasurementsPerRotation = 2000,
                    FieldOfView = 26.9f,
                    VerticalRayAngles = new List<float>(),
                    CenterAngle = 11.45f
                },
                new Template
                {
                    Name = "Lidar128",
                    LaserCount = 128,
                    MinDistance = 0.5f,
                    MaxDistance = 300f,
                    RotationFrequency = 10f,
                    MeasurementsPerRotation = 3272,
                    FieldOfView = 40f,
                    VerticalRayAngles = new List<float>(),
                    CenterAngle = 5f
                }
            };
        }
        */

        /*private static class Properties
        {
            public static readonly int Input = Shader.PropertyToID("_Input");
            public static readonly int Output = Shader.PropertyToID("_Output");
            public static readonly int SinLatitudeAngles = Shader.PropertyToID("_SinLatitudeAngles");
            public static readonly int CosLatitudeAngles = Shader.PropertyToID("_CosLatitudeAngles");
            public static readonly int Index = Shader.PropertyToID("_Index");
            public static readonly int Count = Shader.PropertyToID("_Count");
            public static readonly int LaserCount = Shader.PropertyToID("_LaserCount");
            public static readonly int MeasuresPerRotation = Shader.PropertyToID("_MeasurementsPerRotation");
            public static readonly int Origin = Shader.PropertyToID("_Origin");
            public static readonly int Transform = Shader.PropertyToID("_Transform");
            public static readonly int CameraToWorld = Shader.PropertyToID("_CameraToWorld");
            public static readonly int ScaleDistance = Shader.PropertyToID("_ScaleDistance");
            public static readonly int TexSize = Shader.PropertyToID("_TexSize");
            public static readonly int LongitudeAngles = Shader.PropertyToID("_LongitudeAngles");

            // Custom Shader properties:
            public static readonly int PointsCustom = Shader.PropertyToID("_CustomPoints");
            public static readonly int SectorsCustom = Shader.PropertyToID("_CustomSectors");
            public static readonly int IdxCustom = Shader.PropertyToID("_CustomIdx");
            public static readonly int SizeCustom = Shader.PropertyToID("_CustomSize");
            public static readonly int IndexCustom = Shader.PropertyToID("_CustomIndex");
            public static readonly int YawCustom = Shader.PropertyToID("_CustomYaw");
            public static readonly int PitchCustom = Shader.PropertyToID("_CustomPitch");
        }*/


        public override void Update()
        {
            if (lastFrameCustom != Custom)
            {
                lastFrameCustom = Custom;
                Reset();
                if (!Custom)
                {
                    //ReleaseCustomBuffers(filePassData[passIndex]);
                }
            }

            base.Update();
        }

        protected override void Initialize()
        {
            //RuntimeSettings.Instance.LidarComputeShader = computeShader;

            baseLink = transform.parent.GetComponentInChildren<BaseLink>();
            timeSincePass = Time.time;
            LoadFileAngleData();
            FindMinMaxAngles();
            base.Initialize();
        }

        protected override void Deinitialize()
        {
            foreach (var passData in filePassData.Values)
            {
                ReleaseCustomBuffers(passData);
            }
        }

      
        protected override bool SetComputeParams(CommandBuffer cmd, int kernel)
        {
            bool shouldRender = true;
            shouldRender = base.SetComputeParams(cmd, kernel);
            
            if (Time.time - timeSincePass > timePerPass && anglePassAmount > 1)//
            {
                
                passIndex = (passIndex + 1) % anglePassAmount;
                //CustomReset();
            }

          
            
            
            // Set Custom Properties //
            cmd.SetComputeIntParam(cs, Properties.SectorsCustom, customSectors);
            cmd.SetComputeIntParam(cs, Properties.PointsCustom, customPoints);
            cmd.SetComputeBufferParam(cs, kernel, Properties.SizeCustom, filePassData[passIndex].CustomSizeBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.IndexCustom, filePassData[passIndex].CustomIndexBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.PitchCustom, filePassData[passIndex].CustomPitchBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.YawCustom, filePassData[passIndex].CustomYawBuffer);
            //

            int yDimension = filePassData[passIndex].CustomSize.Max() + 1;
            DispatchThreads = new Vector3Int(customSectors, yDimension, 1);
            //Debug.Log($"HDRPUtilities.GetGroupSize(yDimension, 1): {HDRPUtilities.GetGroupSize(yDimension, 1)}\nyDimension:{yDimension}\nIdx: {idx}");
            /*if (yDimension > 0)
            {
                DispatchThreads = new Vector3Int(customSectors, yDimension, 1); //HDRPUtilities.GetGroupSize(customSectors, 1), HDRPUtilities.GetGroupSize(yDimension, 1), 1);
                //cmd.DispatchCompute(cs, Kernel, 1, yDimension, 1);
                //Debug.Log($"idx:{idx}  yDimension:{yDimension}");
            }
            else
            {
                return false;
            }*/

            return shouldRender;
        }


        private void CustomReset()
        {
           
            
            /*
              indexReset = true;
            StartLatitudeAngle = 90.0f - minAngle;
            EndLatitudeAngle = 90.0f - maxAngle;
            FieldOfView = StartLatitudeAngle - EndLatitudeAngle;
            MaxAngle = Mathf.Max((float)(StartLatitudeAngle - 90.0), (float)(90.0 - EndLatitudeAngle));
            Debug.Log($"Min angle:{minAngle}  Max angle:{maxAngle}  FOV:{FieldOfView}  Start latitude:{StartLatitudeAngle}  End latitude{EndLatitudeAngle}");
            */
            

            // Set Shader Data
          
        }

        private void LoadFileAngleData()
        {
            string path = Application.dataPath;
            //Copy the LIDAR angles file
            try
            {
                // Will not overwrite if the destination file already exists.
                System.IO.File.Copy(path + $"/Carteav/{anglesFile}", path + $"/../simulator_Data/{anglesFile}", true);
            }
            // Catch exception if the file was already copied.
            catch (IOException copyError)
            {
                Console.WriteLine(copyError.Message);
            }

            //Read LIDAR angles file
            GetTwoArraysFromFile(path + $"/Carteav/{anglesFile}");
        }

        private void ReleaseCustomBuffers(PassData passData)
        {
            if (passData.CustomSizeBuffer != null)
            {
                passData.CustomSizeBuffer.Release();
                passData.CustomSizeBuffer = null;
            }

            if (passData.CustomIndexBuffer != null)
            {
                passData.CustomIndexBuffer.Release();
                passData.CustomIndexBuffer = null;
            }

            if (passData.CustomYawBuffer != null)
            {
                passData.CustomYawBuffer.Release();
                passData.CustomYawBuffer = null;
            }

            if (passData.CustomPitchBuffer != null)
            {
                passData.CustomPitchBuffer.Release();
                passData.CustomPitchBuffer = null;
            }
        }


        public void GetTwoArraysFromFile(string filein)
        {
            System.IO.StreamReader file = new System.IO.StreamReader(filein);
            for (int pass = 0; pass < anglePassAmount; pass++)
            {
                string line;
                List<List<float>> p1 = new List<List<float>>(); //Creates new nested List
                List<List<float>> p2 = new List<List<float>>(); //Creates new nested List
                for (int i = 0; i < customSectors; i++)
                {
                    p1.Add(new List<float>()); //Adds new sub List
                    p2.Add(new List<float>()); //Adds new sub List
                }

                int[] lines = new int[customSectors];
                for (int i = 0; i < lines.Length; i++)
                {
                    lines[i] = 0;
                }

                int lineCount = 0;
                

                while ((line = file.ReadLine()) != null)
                {
                    //try 
                    String[] columns = line.Trim().Split(',');
                    float radiansA = float.Parse(columns[0], CultureInfo.InvariantCulture);
                    float degreesB = radiansA * 180f / Mathf.PI;
                    int sectorG =
                        degreesB > 0
                            ? (int)(degreesB / 15f)
                            : (int)(degreesB / 15f) - 1; //int.Parse(columns[7], CultureInfo.InvariantCulture);
                    int positiveSectorH = sectorG < 0 ? sectorG + 24 : sectorG;
                    float degreesOffsetC = sectorG * 15 - degreesB + 90f + 7.5f; //(15 / 2)
                    float radianD_Result1 = degreesOffsetC * Mathf.PI / 180f;
                    float radiansE = 0;
                    radiansE = float.Parse(columns[1], CultureInfo.InvariantCulture);
                    float radianF_Result2 = radiansE + Mathf.PI;
                    p1[positiveSectorH].Add(radianD_Result1);
                    p2[positiveSectorH].Add(radianF_Result2);
                    lines[positiveSectorH]++;
                    lineCount++;
                    // Debug.Log($"Line[{lineCount++}] - A:{radiansA}, B:{degreesB}, C:{degreesOffsetC}, D:{radianD_Result1}, E:{radiansE}, F:{radianF_Result2}, G:{sectorG}, H:{positiveSectorH}.\nPass:{passIndex}");
                    if (anglePassAmount > 1 && anglesPerPass > 0 && lineCount > anglesPerPass) break;
                }

                //catch (Exception e){ Debug.LogError(e.Message);}
                //Debug.Log($"first angle in pass {pass}: {p1[0][0]}");
                int maxPointsPerSector = customPoints;//lines.Max();// ?
                
                filePassData[pass] = new PassData
                {
                    P1 = p1, P2 = p2, 
                    Lines = lines, 
                    CustomYaw = new float[customSectors, maxPointsPerSector],
                    CustomPitch = new float[customSectors, maxPointsPerSector], 
                    CustomIndex = new int[customSectors],
                    CustomSize = new int[customSectors],
                    MaxPointsPerSector = maxPointsPerSector
                };

                ExtractAngles(filePassData[pass]);
            }
        }

        private void ExtractAngles(PassData passData)
        {
            int sum = 0;
            for (int i = 0; i < customSectors; i++)
            {
                for (int j = 0; j < passData.Lines[i]; j++)
                {
                    passData.CustomYaw[i, j] = passData.P1[i][j];
                    passData.CustomPitch[i, j] = passData.P2[i][j];
                }

                passData.CustomSize[i] = passData.Lines[i];
                passData.CustomIndex[i] = sum;
                sum += passData.CustomSize[i];
                //print("i= " + i + " " + size[i] + " " + index[i]);
            }
            
            passData.CustomSizeBuffer = new ComputeBuffer(customSectors, 4);
            passData.CustomIndexBuffer = new ComputeBuffer(customSectors, 4);
            passData.CustomYawBuffer = new ComputeBuffer(customSectors * passData.MaxPointsPerSector, 4);
            passData.CustomPitchBuffer = new ComputeBuffer(customSectors * passData.MaxPointsPerSector, 4);
            passData.CustomSizeBuffer.SetData(passData.CustomSize);
            passData.CustomIndexBuffer.SetData(passData.CustomIndex);
            passData.CustomYawBuffer.SetData(passData.CustomYaw);
            passData.CustomPitchBuffer.SetData(passData.CustomPitch);
            
        }

        private void FindMinMaxAngles()
        {
            float angle;
            foreach (var passData in filePassData.Values)
            {
                foreach (var angleList in passData.P1)
                {
                    foreach (var radianAngle in angleList)
                    {
                        angle = (float)(radianAngle * 180f / Math.PI);
                        minAngle = angle < minAngle ? angle : minAngle;
                        maxAngle = angle > maxAngle ? angle : maxAngle;
                    }
                }
                
                foreach (var angleList in passData.P2)
                {
                    foreach (var radianAngle in angleList)
                    {
                        angle = (float)(radianAngle * 180f / Math.PI);
                        minAngle = angle < minAngle ? angle : minAngle;
                        maxAngle = angle > maxAngle ? angle : maxAngle;
                    }
                }
            }
        }
        
        
             
    /*private void CustomRender(ScriptableRenderContext context, HDCamera hd)
    {
      CommandBuffer cmd = CommandBufferPool.Get();
      SensorPassRenderer.Render(context, cmd, hd, this.renderTarget, this.passId, Color.clear, new Action<CubemapFace>(RenderPointCloud));
      int kernel = this.cs.FindKernel(this.Compensated ? "CubeComputeComp" : "CubeCompute");
      cmd.SetComputeTextureParam(this.cs, kernel, Properties.InputCubemapTexture, renderTarget.ColorHandle);
      cmd.SetComputeBufferParam(this.cs, kernel, Properties.Output, this.PointCloudBuffer);
      cmd.SetComputeBufferParam(this.cs, kernel, Properties.LatitudeAngles, this.LatitudeAnglesBuffer);
      cmd.SetComputeIntParam(this.cs, Properties.LaserCount, this.LaserCount);
      cmd.SetComputeIntParam(this.cs, Properties.MeasuresPerRotation, this.MeasurementsPerRotation);
      cmd.SetComputeVectorParam(this.cs, Properties.Origin, SensorCamera.transform.position);
      cmd.SetComputeMatrixParam(this.cs, Properties.RotMatrix, Matrix4x4.Rotate(((Component) this).transform.rotation));
      cmd.SetComputeMatrixParam(this.cs, Properties.Transform, ((Component) this).transform.worldToLocalMatrix);
      cmd.SetComputeVectorParam(this.cs, Properties.PackedVec, new Vector4(this.MaxDistance, this.DeltaLongitudeAngle, this.MinDistance, 0.0f));
      cmd.DispatchCompute(this.cs, kernel, HDRPUtilities.GetGroupSize(this.MeasurementsPerRotation, 8), HDRPUtilities.GetGroupSize(this.LaserCount, 8), 1);
      context.ExecuteCommandBuffer(cmd);
      cmd.Clear();
      CommandBufferPool.Release(cmd);

      void RenderPointCloud(CubemapFace face) => PointCloudManager.RenderLidar(context, cmd, hd, this.renderTarget.ColorHandle, this.renderTarget.DepthHandle, face);
    }

   

    private void DifferentUpdate()
    {
      if (this.LaserCount != this.CurrentLaserCount || this.MeasurementsPerRotation != this.CurrentMeasurementsPerRotation || !Mathf.Approximately(this.FieldOfView, this.CurrentFieldOfView) || !Mathf.Approximately(this.CenterAngle, this.CurrentCenterAngle) || !Mathf.Approximately(this.MinDistance, this.CurrentMinDistance) || !Mathf.Approximately(this.MaxDistance, this.CurrentMaxDistance) || !this.VerticalRayAngles.SequenceEqual<float>((IEnumerable<float>) this.CurrentVerticalRayAngles))
        this.Reset();
      this.SensorCamera.fieldOfView = this.FieldOfView;
      this.SensorCamera.nearClipPlane = this.MinDistance;
      this.SensorCamera.farClipPlane = this.MaxDistance;
      this.CheckTexture();
      this.CheckCapture();
    }
    private void CheckTexture()
    {
        if (this.renderTarget != null && (!this.renderTarget.IsCube || !this.renderTarget.IsValid(this.CubemapSize, this.CubemapSize)))
        {
            this.renderTarget.Release();
            this.renderTarget = (SensorRenderTarget) null;
        }
        if (this.renderTarget != null)
            return;
        this.renderTarget = SensorRenderTarget.CreateCube(this.CubemapSize, this.CubemapSize, this.faceMask);
        this.SensorCamera.targetTexture = (RenderTexture) null;
    }

    private void RenderCamera() => this.SensorCamera.Render();

    private void CheckCapture()
    {
        if ((double) Time.time < (double) this.NextCaptureTime)
            return;
        this.RenderCamera();
        this.SendMessage();
        if ((double) this.NextCaptureTime < (double) Time.time - (double) Time.deltaTime)
            this.NextCaptureTime = Time.time + 1f / this.RotationFrequency;
        else
            this.NextCaptureTime += 1f / this.RotationFrequency;
    }*/
    }
}