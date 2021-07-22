// Decompiled with JetBrains decompiler
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
using Object = UnityEngine.Object;

namespace Simulator.Sensors
{
  [SensorType("Lidar", new[] {typeof (PointCloudData)})]
  public class CarteavLidarSensor : LidarSensorBase
  {
    /* Custom Properties */
    public bool Custom;
    [SerializeField]
    private ComputeShader CustomLidarComputeShader;
    private static int CustomPoints = 500000;
    private static int CustomSectors = 24;
    private static float[,] CustomYaw = new float[CustomSectors, CustomPoints]; 
    private static float[,] CustomPitch = new float[CustomSectors , CustomPoints]; 
    private static int[] CustomSize = new int[CustomSectors];
    private static int[] CustomIndex = new int[CustomSectors];
    private ComputeBuffer CustomSizeBuffer;
    private ComputeBuffer CustomIndexBuffer;
    private ComputeBuffer CustomYawBuffer;
    private ComputeBuffer CustomPitchBuffer;
    private bool lastFrameCustom;
    //
    
    
    public void ApplyTemplate()
    {
      Template template = Template.Templates[TemplateIndex];
      LaserCount =  template.LaserCount;
      MinDistance =   template.MinDistance;
      MaxDistance =   template.MaxDistance;
      RotationFrequency =   template.RotationFrequency;
      MeasurementsPerRotation =  template.MeasurementsPerRotation;
      FieldOfView =   template.FieldOfView;
      VerticalRayAngles =  new List<float>(template.VerticalRayAngles);
      CenterAngle =   template.CenterAngle;
    }

    public override void Reset()
    {
      Active.ForEach(req => req.TextureSet.Release());
      Active.Clear();
      foreach (SensorRenderTarget availableRenderTexture in AvailableRenderTextures)
        availableRenderTexture.Release();
      AvailableRenderTextures.Clear();
      foreach (Object availableTexture in AvailableTextures)
        Destroy(availableTexture);
      AvailableTextures.Clear();
      if (PointCloudBuffer != null)
      {
        PointCloudBuffer.Release();
        PointCloudBuffer = null;
      }
      if (CosLatitudeAnglesBuffer != null)
      {
        CosLatitudeAnglesBuffer.Release();
        CosLatitudeAnglesBuffer = null;
      }
      if (SinLatitudeAnglesBuffer != null)
      {
        SinLatitudeAnglesBuffer.Release();
        SinLatitudeAnglesBuffer = null;
      }
      AngleStart =  0.0f;
      if (VerticalRayAngles.Count == 0)
      {
        MaxAngle =  ( Mathf.Abs(CenterAngle) + FieldOfView / 2.0f);
        StartLatitudeAngle =  (90.0f + MaxAngle);
        if (CenterAngle < 0.0)
          StartLatitudeAngle =  (StartLatitudeAngle - (MaxAngle * 2.0f - FieldOfView));
        EndLatitudeAngle = StartLatitudeAngle - FieldOfView;
      }
      else
      {
        LaserCount =  VerticalRayAngles.Count;
        StartLatitudeAngle =  90.0f -  VerticalRayAngles.Min();
        EndLatitudeAngle =  90.0f -  VerticalRayAngles.Max();
        FieldOfView = StartLatitudeAngle - EndLatitudeAngle;
        MaxAngle =   Mathf.Max((float) (StartLatitudeAngle - 90.0), (float) (90.0 - EndLatitudeAngle));
      }
      float num1 = 97.5f;
      SinStartLongitudeAngle =   Mathf.Sin(num1 * ((float) Math.PI / 180f));
      CosStartLongitudeAngle =   Mathf.Cos(num1 * ((float) Math.PI / 180f));
      MaxAngle =   Mathf.Max(MaxAngle, Mathf.Max(CalculateFovAngle(StartLatitudeAngle, num1), CalculateFovAngle(EndLatitudeAngle, num1)));
      SinLatitudeAngles =  new float[LaserCount];
      CosLatitudeAngles =  new float[LaserCount];
      // Custom length
      int length = CustomPoints;//LaserCount * MeasurementsPerRotation;
      PointCloudBuffer =  new ComputeBuffer(length, UnsafeUtility.SizeOf<Vector4>());
      CosLatitudeAnglesBuffer =  new ComputeBuffer(LaserCount, 4);
      SinLatitudeAnglesBuffer =  new ComputeBuffer(LaserCount, 4);
      if (PointCloudMaterial != null)
        PointCloudMaterial.SetBuffer("_PointCloud", PointCloudBuffer);
      Points =  new Vector4[length];
      CurrentLaserCount = LaserCount;
      CurrentMeasurementsPerRotation = MeasurementsPerRotation;
      CurrentFieldOfView = FieldOfView;
      CurrentVerticalRayAngles =  new List<float>(VerticalRayAngles);
      CurrentCenterAngle = CenterAngle;
      CurrentMinDistance = MinDistance;
      CurrentMaxDistance = MaxDistance;
      IgnoreNewRquests =  0.0f;
      if (VerticalRayAngles.Count == 0)
      {
        float num2 = FieldOfView / LaserCount;
        int index = 0;
        float startLatitudeAngle = StartLatitudeAngle;
        while (index < LaserCount)
        {
          SinLatitudeAngles[index] =  Mathf.Sin(startLatitudeAngle * ((float) Math.PI / 180f));
          CosLatitudeAngles[index] =  Mathf.Cos(startLatitudeAngle * ((float) Math.PI / 180f));
          ++index;
          startLatitudeAngle -= num2;
        }
      }
      else
      {
        for (int index = 0; index < LaserCount; ++index)
        {
          SinLatitudeAngles[index] = Mathf.Sin((float) ((90.0 -  VerticalRayAngles[index]) * (Math.PI / 180.0)));
          CosLatitudeAngles[index] =  Mathf.Cos((float) ((90.0 -  VerticalRayAngles[index]) * (Math.PI / 180.0)));
        }
      }
      CosLatitudeAnglesBuffer.SetData(CosLatitudeAngles);
      SinLatitudeAnglesBuffer.SetData(SinLatitudeAngles);
      DeltaLongitudeAngle =  (15.0f / Mathf.CeilToInt((15.0f / (360.0f /  MeasurementsPerRotation))));
      RenderTextureHeight =  (16 * Mathf.CeilToInt((float) (2.0 * MaxAngle *  (float) LaserCount / FieldOfView)));
      RenderTextureWidth =  (8 * Mathf.CeilToInt((float) (15.0 / (360.0 /  (float) MeasurementsPerRotation))));
      float num3 = (float) (2.0 * MinDistance) * Mathf.Tan(0.1308997f);
      float num4 = (float) (2.0 * MinDistance) * Mathf.Tan((float) (MaxAngle * (Math.PI / 180.0)));
      XScale =  ( num3 /  RenderTextureWidth);
      YScale =  ( num4 /  RenderTextureHeight);
      float num5 = 1f / Mathf.Tan((float) (MaxAngle * (Math.PI / 180.0)));
      float num6 = 1f / Mathf.Tan(0.1308997f);
      float num7 = (MaxDistance + MinDistance) / (MinDistance - MaxDistance);
      float num8 = (float) (2.0 * MaxDistance * MinDistance / (MinDistance - MaxDistance));
      Matrix4x4 matrix4x4 = new Matrix4x4(new Vector4(num6, 0.0f, 0.0f, 0.0f), new Vector4(0.0f, num5, 0.0f, 0.0f), new Vector4(0.0f, 0.0f, num7, -1f), new Vector4(0.0f, 0.0f, num8, 0.0f));
      // ISSUE: explicit constructor call
      //((Matrix4x4) ref matrix4x4).\u002Ector(new Vector4(num6, 0.0f, 0.0f, 0.0f), new Vector4(0.0f, num5, 0.0f, 0.0f), new Vector4(0.0f, 0.0f, num7, -1f), new Vector4(0.0f, 0.0f, num8, 0.0f));
      SensorCamera.nearClipPlane = MinDistance;
      SensorCamera.farClipPlane = MaxDistance;
      SensorCamera.projectionMatrix = matrix4x4;

      if (Custom)
      {
        CustomReset();
      }
    }

    protected override void EndReadRequest(CommandBuffer cmd, ReadRequest req)
    {
      //UnityEngine.Debug.DebugBreak();
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
      cmd.SetComputeVectorParam(cs, Properties.ScaleDistance, new Vector4(XScale, YScale, MinDistance, MaxDistance));
      cmd.SetComputeVectorParam(cs, Properties.TexSize, new Vector4(RenderTextureWidth, RenderTextureHeight, 1f / RenderTextureWidth, 1f / RenderTextureHeight));
      cmd.SetComputeVectorParam(cs, Properties.LongitudeAngles, new Vector4(SinStartLongitudeAngle, CosStartLongitudeAngle, DeltaLongitudeAngle, 0.0f));
      
      if (Custom)
      {
        CustomDispatch(cmd, req, kernel);
      }
      else
      {
        cmd.DispatchCompute(cs, kernel, HDRPUtilities.GetGroupSize(req.Count, 8), HDRPUtilities.GetGroupSize(LaserCount, 8), 1);
      }
    }

    protected override void SendMessage()
    {
      if (Bridge == null || Bridge.Status != Status.Connected)
        return;
      Matrix4x4 worldToLocal = LidarTransform;
      if (Compensated != null)
        worldToLocal = worldToLocal * transform.worldToLocalMatrix;
      PointCloudBuffer.GetData(Points);
      Task.Run(() =>
      {
        // ISSUE: variable of the null type
        var publish = Publish;
        PointCloudData pointCloudData = new PointCloudData();
        pointCloudData.Name = Name;
        pointCloudData.Frame = Frame;
        pointCloudData.Time =  SimulatorManager.Instance.CurrentTime;
        uint sendSequence = SendSequence;
        SendSequence =  sendSequence + 1;
        pointCloudData.Sequence =  sendSequence;
        pointCloudData.LaserCount = CurrentLaserCount;
        pointCloudData.Transform =  worldToLocal;
        pointCloudData.Points = Points;
        publish.Invoke(pointCloudData);
      });
    }

    private void OnValidate()
    {
      if (TemplateIndex == null)
        return;
      Template template = Template.Templates[TemplateIndex];
      if (LaserCount == template.LaserCount && MinDistance ==  template.MinDistance && (MaxDistance ==  template.MaxDistance && RotationFrequency ==  template.RotationFrequency) && (MeasurementsPerRotation == template.MeasurementsPerRotation && FieldOfView ==  template.FieldOfView && (CenterAngle ==  template.CenterAngle && VerticalRayAngles.SequenceEqual(template.VerticalRayAngles))))
        return;
      TemplateIndex =  0;
    }

    public struct Template
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
        new Template { Name = "Custom",
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

    private static class Properties
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
    }


    public override void Update()
    {
      if (lastFrameCustom != Custom)
      {
        lastFrameCustom = Custom;
        Reset();
        if (!Custom)
        {
          CustomReset();
        }
        else
        {
          CurrentIndex = 0;
        }
      }
      base.Update();
    }

    public override void Init()
    {
      transform.localPosition = new Vector3(0.6f, 1.46f, 3f);
      transform.localRotation = Quaternion.identity;
      
      RuntimeSettings.Instance.LidarComputeShader = CustomLidarComputeShader;
      base.Init();
    }

    private void CustomDispatch(CommandBuffer cmd, ReadRequest req, int kernel)
    {
      int idx = req.Index / 15;
      
      // Set Custom Properties //
      cmd.SetComputeIntParam(cs, Properties.IdxCustom, idx);
      cmd.SetComputeIntParam(cs, Properties.PointsCustom, CustomPoints);
      cmd.SetComputeIntParam(cs, Properties.SectorsCustom, CustomSectors);
      cmd.SetComputeBufferParam(cs, kernel, Properties.SizeCustom, CustomSizeBuffer);
      cmd.SetComputeBufferParam(cs, kernel, Properties.IndexCustom, CustomIndexBuffer);
      cmd.SetComputeBufferParam(cs, kernel, Properties.PitchCustom, CustomPitchBuffer);
      cmd.SetComputeBufferParam(cs, kernel, Properties.YawCustom, CustomYawBuffer);
      //
      
      int yDimension = CustomSize[idx];
      //Debug.Log($"HDRPUtilities.GetGroupSize(yDimension, 1): {HDRPUtilities.GetGroupSize(yDimension, 1)}\nyDimension:{yDimension}\nIdx: {idx}");
      if (yDimension > 0)
      {
        cmd.DispatchCompute(cs, kernel, 1, yDimension, 1);
      }
    }
    
    
    private void CustomReset()
    {
      if (CustomSizeBuffer != null)
      {
        CustomSizeBuffer.Release();
        CustomSizeBuffer = null;
      }
      if (CustomIndexBuffer != null)
      {
        CustomIndexBuffer.Release();
        CustomIndexBuffer = null;
      }
      if (CustomYawBuffer != null)
      {
        CustomYawBuffer.Release();
        CustomYawBuffer = null;
      }
      if (CustomPitchBuffer != null)
      {
        CustomPitchBuffer.Release();
        CustomPitchBuffer = null;
      }
      
      
      // Set Shader Data
      CustomSizeBuffer = new ComputeBuffer(CustomSectors, 4);
      CustomIndexBuffer = new ComputeBuffer(CustomSectors, 4);
      CustomYawBuffer = new ComputeBuffer(CustomSectors * CustomPoints, 4);
      CustomPitchBuffer = new ComputeBuffer(CustomSectors * CustomPoints, 4);
      CustomSizeBuffer.SetData(CustomSize);
      CustomIndexBuffer.SetData(CustomIndex);
      CustomYawBuffer.SetData(CustomYaw);
      CustomPitchBuffer.SetData(CustomPitch);
      
      
      string path = Application.dataPath;
      //Copy the LIDAR angles file
      try
      {
        // Will not overwrite if the destination file already exists.
        System.IO.File.Copy(path + "/rs_m1.csv", path + "/../simulator_Data/rs_m1.csv", true);
      }
      // Catch exception if the file was already copied.
      catch (IOException copyError)
      {
        Console.WriteLine(copyError.Message);
      }

      //Read LIDAR angles file
      getTwoArraysFromFile_(path + "/rs_m1.csv", ref CustomYaw, ref CustomPitch, ref CustomSize, ref CustomIndex);
    }
    
    public static void getTwoArraysFromFile_(string filein, ref float[,] yaw_, ref float[,] pitch_, ref int[] size_, ref int[] Index_)
    {
      string line;
      List<List<float>> p1_= new List<List<float>>(); //Creates new nested List
      List<List<float>> p2_= new List<List<float>>(); //Creates new nested List
      for(int i=0;i<CustomSectors;i++)
      {
        p1_.Add(new List<float>()); //Adds new sub List
        p2_.Add(new List<float>()); //Adds new sub List
      }	 
      int[] lines_ = new int[CustomSectors];
      for(int i = 0; i < lines_.Length; i++)
        lines_[i] = 0;
      int sector;
	    
      //int lines=0;
      //List<float> p1 = new List<float>();
      //List<float> p2 = new List<float>();

      System.IO.StreamReader file = new System.IO.StreamReader(filein);
      while ((line = file.ReadLine()) != null)
        try {
          String[] parms = line.Trim().Split(',');
          sector = int.Parse(parms[7], CultureInfo.InvariantCulture);
          p1_[sector].Add(float.Parse(parms[3], CultureInfo.InvariantCulture));
          p2_[sector].Add(float.Parse(parms[5], CultureInfo.InvariantCulture));
          lines_[sector]++;
          //p1.Add(float.Parse(parms[0], CultureInfo.InvariantCulture));       
          //p2.Add(float.Parse(parms[1], CultureInfo.InvariantCulture));
          //lines++;
        }
        catch { }
      int sum=0;
      for(int i=0;i<CustomSectors;i++)
      {
        for(int j=0;j<lines_[i];j++)
        {			    
          yaw_[i , j] = p1_[i][j];
          pitch_[i , j] = p2_[i][j];
        }
        size_[i] = lines_[i];
        Index_[i] = sum;
        sum += size_[i];
        print("i= " + i + " " + size_[i] + " " + Index_[i]);
        //yaw = p1.ToArray();
        //pitch = p2.ToArray();
        //size = lines;
      }
    }
  }
}
