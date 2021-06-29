/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
using UnityEngine;
 using System;
using System.IO;
using System.Threading;
using System.Globalization;
using System.Linq;
using System.Threading.Tasks;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using Simulator.Bridge;
using Simulator.Utilities;
using WebSocketSharp;
using PointCloudData = Simulator.Bridge.Data.PointCloudData;

namespace Simulator.Sensors.Carteav
{
    [SensorType("Lidar", new[] { typeof(PointCloudData) })]
    public partial class LidarSensor : LidarSensorBase
    {
        static public int points = 500000;
        static public int sectors = 24;
        static public float[,] yaw_ = new float[sectors,points]; 
        static public float[,] pitch_ = new float[sectors,points]; 
        static public int[] size_ = new int[sectors];
        static public int[] Index_ = new int[sectors];

        protected float SinDeltaLongitudeAngle;
        protected float CosDeltaLongitudeAngle;
        
        public void ApplyTemplate()
        {
            var values = Simulator.Sensors.LidarSensor.Template.Templates[TemplateIndex];
            LaserCount = values.LaserCount;
            MinDistance = values.MinDistance;
            MaxDistance = values.MaxDistance;
            RotationFrequency = values.RotationFrequency;
            MeasurementsPerRotation = values.MeasurementsPerRotation;
            FieldOfView = values.FieldOfView;
            VerticalRayAngles = new List<float>(values.VerticalRayAngles);
            CenterAngle = values.CenterAngle;
        }

	public static void getTwoArraysFromFile_(string filein, ref float[,] yaw_, ref float[,] pitch_, ref int[] size_, ref int[] Index_)
	{
	    string line;
	    List<List<float>> p1_= new List<List<float>>(); //Creates new nested List
	    List<List<float>> p2_= new List<List<float>>(); //Creates new nested List
	    for(int i=0;i<sectors;i++)
	    {
		    p1_.Add(new List<float>()); //Adds new sub List
		    p2_.Add(new List<float>()); //Adds new sub List
	    }	 
	    int[] lines_ = new int[sectors];
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
	    for(int i=0;i<sectors;i++)
	    {
		    for(int j=0;j<lines_[i];j++)
		    {			    
			    yaw_[i,j] = p1_[i][j];
			    pitch_[i,j] = p2_[i][j];
		    }
		    size_[i] = lines_[i];
		    Index_[i] = sum;
		    sum += size_[i];
		    //print("i= " + i + " " + size_[i] + " " + Index_[i]);
		    //yaw = p1.ToArray();
		    //pitch = p2.ToArray();
		    //size = lines;
	    }
	}

        public override void Reset()
        {
            Active.ForEach(req =>
            {
                //req.Readback.WaitForCompletion();
                req.TextureSet.Release();
            });
            Active.Clear();

            /*Jobs.ForEach(job => job.Complete());
            Jobs.Clear();*/

            foreach (var tex in AvailableRenderTextures)
            {
                tex.Release();
            };
            AvailableRenderTextures.Clear();

            foreach (var tex in AvailableTextures)
            {
                Destroy(tex);
            };
            AvailableTextures.Clear();

            if (PointCloudBuffer != null)
            {
                PointCloudBuffer.Release();
                PointCloudBuffer = null;
            }

            /*if (Points.IsCreated)
            {
                Points.Dispose();
            }*/

            AngleStart = 0.0f;
            // Assuming center of view frustum is horizontal, find the vertical FOV (of view frustum) that can encompass the tilted Lidar FOV.
            // "MaxAngle" is half of the vertical FOV of view frustum.
            if (VerticalRayAngles.Count == 0)
            {
                MaxAngle = Mathf.Abs(CenterAngle) + FieldOfView / 2.0f;

                StartLatitudeAngle = 90.0f + MaxAngle;
                //If the Lidar is tilted up, ignore lower part of the vertical FOV.
                if (CenterAngle < 0.0f)
                {
                    StartLatitudeAngle -= MaxAngle * 2.0f - FieldOfView;
                }
                EndLatitudeAngle = StartLatitudeAngle - FieldOfView;
            }
            else
            {
                LaserCount = VerticalRayAngles.Count;
                StartLatitudeAngle = 90.0f - VerticalRayAngles.Min();
                EndLatitudeAngle = 90.0f - VerticalRayAngles.Max();
                FieldOfView = StartLatitudeAngle - EndLatitudeAngle;
                MaxAngle = Mathf.Max(StartLatitudeAngle - 90.0f, 90.0f - EndLatitudeAngle);
            }

            float startLongitudeAngle = 90.0f + HorizontalAngleLimit / 2.0f;
            SinStartLongitudeAngle = Mathf.Sin(startLongitudeAngle * Mathf.Deg2Rad);
            CosStartLongitudeAngle = Mathf.Cos(startLongitudeAngle * Mathf.Deg2Rad);

            // The MaxAngle above is the calculated at the center of the view frustum.
            // Because the scan curve for a particular laser ray is a hyperbola (intersection of a conic surface and a vertical plane),
            // the vertical FOV should be enlarged toward left and right ends.
            float startFovAngle = CalculateFovAngle(StartLatitudeAngle, startLongitudeAngle);
            float endFovAngle = CalculateFovAngle(EndLatitudeAngle, startLongitudeAngle);
            MaxAngle = Mathf.Max(MaxAngle, Mathf.Max(startFovAngle, endFovAngle));

            // Calculate sin/cos of latitude angle of each ray.
            /*if (SinLatitudeAngles.IsCreated)
            {
                SinLatitudeAngles.Dispose();
            }
            if (CosLatitudeAngles.IsCreated)
            {
                CosLatitudeAngles.Dispose();
            }*/
            SinLatitudeAngles = new float[LaserCount];
            CosLatitudeAngles = new float[LaserCount];;


            int totalCount = points;// LaserCount * MeasurementsPerRotation;
            PointCloudBuffer = new ComputeBuffer(totalCount, UnsafeUtility.SizeOf<Vector4>());
            PointCloudMaterial?.SetBuffer("_PointCloud", PointCloudBuffer);

            Points = new Vector4[totalCount];

            CurrentLaserCount = LaserCount;
            CurrentMeasurementsPerRotation = MeasurementsPerRotation;
            CurrentFieldOfView = FieldOfView;
            CurrentVerticalRayAngles = new List<float>(VerticalRayAngles);
            CurrentCenterAngle = CenterAngle;
            CurrentMinDistance = MinDistance;
            CurrentMaxDistance = MaxDistance;

            IgnoreNewRquests = 0;

            // If VerticalRayAngles array is not provided, use uniformly distributed angles.
            if (VerticalRayAngles.Count == 0)
            {
                float deltaLatitudeAngle = FieldOfView / LaserCount;
                int index = 0;
                float angle = StartLatitudeAngle;
                while (index < LaserCount)
                {
                    SinLatitudeAngles[index] = Mathf.Sin(angle * Mathf.Deg2Rad);
                    CosLatitudeAngles[index] = Mathf.Cos(angle * Mathf.Deg2Rad);
                    index++;
                    angle -= deltaLatitudeAngle;
                }
            }
            else
            {
                for (int index = 0; index < LaserCount; index++)
                {
                    SinLatitudeAngles[index] = Mathf.Sin((90.0f - VerticalRayAngles[index]) * Mathf.Deg2Rad);
                    CosLatitudeAngles[index] = Mathf.Cos((90.0f - VerticalRayAngles[index]) * Mathf.Deg2Rad);
                }
            }

            int count = Mathf.CeilToInt(HorizontalAngleLimit / (360.0f / MeasurementsPerRotation));
            float deltaLongitudeAngle = (float)HorizontalAngleLimit / (float)count;
            SinDeltaLongitudeAngle = Mathf.Sin(deltaLongitudeAngle * Mathf.Deg2Rad);
            CosDeltaLongitudeAngle = Mathf.Cos(deltaLongitudeAngle * Mathf.Deg2Rad);

            // Enlarged the texture by some factors to mitigate alias.
            RenderTextureHeight = 16 * Mathf.CeilToInt(2.0f * MaxAngle * LaserCount / FieldOfView);
            RenderTextureWidth = 8 * Mathf.CeilToInt(HorizontalAngleLimit / (360.0f / MeasurementsPerRotation));

            // View frustum size at the near plane.
            float frustumWidth = 2 * MinDistance * Mathf.Tan(HorizontalAngleLimit / 2.0f * Mathf.Deg2Rad);
            float frustumHeight = 2 * MinDistance * Mathf.Tan(MaxAngle * Mathf.Deg2Rad);
            XScale = frustumWidth / RenderTextureWidth;
            YScale = frustumHeight / RenderTextureHeight;

            // construct custom aspect ratio projection matrix
            // math from https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix

            float v = 1.0f / Mathf.Tan(MaxAngle * Mathf.Deg2Rad);
            float h = 1.0f / Mathf.Tan(HorizontalAngleLimit * Mathf.Deg2Rad / 2.0f);
            float a = (MaxDistance + MinDistance) / (MinDistance - MaxDistance);
            float b = 2.0f * MaxDistance * MinDistance / (MinDistance - MaxDistance);

            var projection = new Matrix4x4(
                new Vector4(h, 0, 0, 0),
                new Vector4(0, v, 0, 0),
                new Vector4(0, 0, a, -1),
                new Vector4(0, 0, b, 0));

            SensorCamera.nearClipPlane = MinDistance;
            SensorCamera.farClipPlane = MaxDistance;
            SensorCamera.projectionMatrix = projection;

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
            getTwoArraysFromFile_(path + "/rs_m1.csv", ref yaw_, ref pitch_, ref size_, ref Index_);
        }

        struct UpdatePointCloudJob : IJob
        {
            [ReadOnly, DeallocateOnJobCompletion]
            public NativeArray<byte> Input;

            [WriteOnly, NativeDisableContainerSafetyRestriction]
            public Vector4[]Output;

            // Index of frames
            public int Index;
            // Number of measurements (vertical line) per view frustum
            public int Count;
            // Origin of the camera coordinate system
            public Vector3 Origin;

            //[ReadOnly, NativeDisableContainerSafetyRestriction]
            public float[] SinLatitudeAngles;
            //[ReadOnly, NativeDisableContainerSafetyRestriction]
            public  float[]  CosLatitudeAngles;

            public float SinStartLongitudeAngle;
            public float CosStartLongitudeAngle;
            public float SinDeltaLongitudeAngle;
            public float CosDeltaLongitudeAngle;
            // Scales between world coordinates and texture coordinates
            public float XScale;
            public float YScale;

            public Matrix4x4 Transform;
            public Matrix4x4 CameraToWorldMatrix;

            // Number of laser rays in one measurement (vertical line)
            public int LaserCount;
            // Number of measurements in a whole rotation round
            public int MeasurementsPerRotation;
            // Size of render texture of the camera
            public int TextureWidth;
            public int TextureHeight;
            // Near plane of view frustum
            public float MinDistance;
            // Far plane of view frustum
            public float MaxDistance;

            public bool Compensated;


            public static float DecodeFloatRGB(byte r, byte g, byte b)
            {
                return (r / 255.0f) + (g / 255.0f) / 255.0f + (b / 255.0f) / 65025.0f;
            }

            public void Execute()
            {
                // In the following loop, x/y are in texture space, and xx/yy are in world space
                int idx = Index / 15; //idx [0...23] - 24 sectors make 360 degrees, each sector is 0..15 degrees and with bias 83..98 degrees
                for (int i = 0; i < size_[idx]; i++) //size_ - number of point in each sector, i - each point
                {
                    float sinLatitudeAngle = Mathf.Sin(pitch_[idx, i]);
                    float cosLatitudeAngle = Mathf.Cos(pitch_[idx, i]);

                    int indexOffset = 0;
                    float dy = sinLatitudeAngle;
                    float rProjected = cosLatitudeAngle;

                    float sinLongitudeAngle = Mathf.Sin(yaw_[idx, i]);
                    float cosLongitudeAngle = Mathf.Cos(yaw_[idx, i]);

                    float dz = rProjected * sinLongitudeAngle;
                    float dx = rProjected * cosLongitudeAngle;

                    float scale = MinDistance / dz;

                    float xx = dx * scale;
                    float yy = dy * scale;

                    int x = (int)(xx / XScale + TextureWidth / 2);
                    int y = (int)(yy / YScale + TextureHeight / 2);

                    int yOffset = y * TextureWidth * 4;

                    float distance;
                    if (x < 0 || x >= TextureWidth || y < 0 || y >= TextureHeight)
                        distance = 0;
                    else
                    {
                        byte r = Input[yOffset + x * 4 + 0];
                        byte g = Input[yOffset + x * 4 + 1];
                        byte b = Input[yOffset + x * 4 + 2];
                        distance = 2.0f * DecodeFloatRGB(r, g, b);
                    }
                    int index = indexOffset + Index_[idx] + i;
                    if (distance == 0)
                        Output[index] = Vector4.zero;
                    else
                    {
                        byte a = Input[yOffset + x * 4 + 3];
                        float intensity = a / 255.0f;

                        // Note that CameraToWorldMatrix follows OpenGL convention, i.e. camera is facing negative Z axis.
                        // So we have "z" component of the direction as "-MinDistance".
                        Vector3 dir = CameraToWorldMatrix.MultiplyPoint3x4(new Vector3(xx, yy, -MinDistance)) - Origin;
                        var position = Origin + dir.normalized * distance * MaxDistance;

                        //if (!Compensated)
                        //{
                        //    position = Transform.MultiplyPoint3x4(position);
                        //}
                        Output[index] = new Vector4(position.x, position.y, position.z, intensity);
                    }
                }
                //Sync for point cloud publisher on the last sector (23) end
                if (idx == 23) Count = 15;
                else Count = 0;
            }
		}	     

             /*public void Execute()
             {
                 // In the following loop, x/y are in texture space, and xx/yy are in world space
                 for (int j = 0; j < LaserCount; j++)
                 {
                     float sinLatitudeAngle = SinLatitudeAngles[j];
                     float cosLatitudeAngle = CosLatitudeAngles[j];

                     //print(sinLatitudeAngle + " " + cosLatitudeAngle);

                     int indexOffset = j * MeasurementsPerRotation;

                     float dy = cosLatitudeAngle;
                     float rProjected = sinLatitudeAngle;

                     float sinLongitudeAngle = SinStartLongitudeAngle;
                     float cosLongitudeAngle = CosStartLongitudeAngle;
			
		      //print(sinLongitudeAngle + " " + cosLongitudeAngle);
		      
                     for (int i = 0; i < Count; i++)
                     {
                         float dz = rProjected * sinLongitudeAngle;
                         float dx = rProjected * cosLongitudeAngle;
			  //print("dz: " + dz + " dx: " + dx);
                         float scale = MinDistance / dz;
                         //print(scale);
                         float xx = dx * scale;
                         float yy = dy * scale;
                         //print("xx: " + xx + " yy: " + yy);
                         int x = (int)(xx / XScale + TextureWidth / 2);
                         int y = (int)(yy / YScale + TextureHeight / 2);
                         int yOffset = y * TextureWidth * 4;
			  //print("x: " + x + " y: " + y);
                         float distance;
                         if (x < 0 || x >= TextureWidth || y < 0 || y >= TextureHeight)
                         {
                             distance = 0;
                         }
                         else
                         {
                             byte r = Input[yOffset + x * 4 + 0];
                             byte g = Input[yOffset + x * 4 + 1];
                             byte b = Input[yOffset + x * 4 + 2];
                             distance = 2.0f * DecodeFloatRGB(r, g, b);
                         }

			  if (Index == 0)
			  {
                         int index = indexOffset + (Index + i) % MeasurementsPerRotation;
                         if (distance == 0)
                         {
                             Output[index] = Vector4.zero;
                         }
                         else
                         {
                             byte a = Input[yOffset + x * 4 + 3];
                             float intensity = a / 255.0f;

                             // Note that CameraToWorldMatrix follows OpenGL convention, i.e. camera is facing negative Z axis.
                             // So we have "z" component of the direction as "-MinDistance".
                             Vector3 dir = CameraToWorldMatrix.MultiplyPoint3x4(new Vector3(xx, yy, -MinDistance)) - Origin;
                             var position = Origin + dir.normalized * distance * MaxDistance;

                             if (!Compensated)
                             {
                                 position = Transform.MultiplyPoint3x4(position);
                             }
                             Output[index] = new Vector4(position.x, position.y, position.z, intensity);
                             //print("x: " + position.x + " y: " + position.y);
                         }
			  }
                         // We will update longitudeAngle as "longitudeAngle -= DeltaLogitudeAngle".
                         // cos/sin of the new longitudeAngle can be calculated using old cos/sin of logitudeAngle
                         // and cos/sin of DeltaLogitudeAngle (which is constant) via "angle addition and subtraction theorems":
                         // sin(a + b) = sin(a) * cos(b) + cos(a) * sin(b)
                         // cos(a + b) = cos(a) * cos(b) - sin(a) * sin(b)
                         float sinNewLongitudeAngle = sinLongitudeAngle * CosDeltaLongitudeAngle - cosLongitudeAngle * SinDeltaLongitudeAngle;
                         float cosNewLongitudeAngle = cosLongitudeAngle * CosDeltaLongitudeAngle + sinLongitudeAngle * SinDeltaLongitudeAngle;
                         sinLongitudeAngle = sinNewLongitudeAngle;
                         cosLongitudeAngle = cosNewLongitudeAngle;
                     }
                 }
             }
        }*/
        
        protected override void EndReadRequest(UnityEngine.Rendering.CommandBuffer cmd, ReadRequest req)
        {
            EndReadMarker.Begin();

            var updateJob = new UpdatePointCloudJob()
            {
                //Input = new NativeArray<byte>(req.TextureSet.DepthTexture.ToByteArray<byte>(ByteOrder.Big), Allocator.TempJob), **********
                Output = Points,

                Index = req.Index,
                Count = req.Count,
                Origin = req.Origin,

                Transform = req.Transform,
                CameraToWorldMatrix = req.CameraToWorldMatrix,

                SinLatitudeAngles = SinLatitudeAngles,
                CosLatitudeAngles = CosLatitudeAngles,

                SinStartLongitudeAngle = SinStartLongitudeAngle,
                CosStartLongitudeAngle = CosStartLongitudeAngle,
                SinDeltaLongitudeAngle = SinDeltaLongitudeAngle,
                CosDeltaLongitudeAngle = CosDeltaLongitudeAngle,
                XScale = XScale,
                YScale = YScale,

                LaserCount = CurrentLaserCount,
                MeasurementsPerRotation = CurrentMeasurementsPerRotation,
                TextureWidth = RenderTextureWidth,
                TextureHeight = RenderTextureHeight,

                MinDistance = MinDistance,
                MaxDistance = MaxDistance,

                Compensated = Compensated,
            };

            EndReadMarker.End();

            //return updateJob.Schedule();
        }

        protected override void SendMessage()
        {
            if (Bridge != null && Bridge.Status == Status.Connected)
            {
                var worldToLocal = LidarTransform;
                if (Compensated)
                {
                    worldToLocal = worldToLocal * transform.worldToLocalMatrix;
                }

                Task.Run(() =>
                {
                    Publish(new PointCloudData()
                    {
                        Name = Name,
                        Frame = Frame,
                        Time = SimulatorManager.Instance.CurrentTime,
                        Sequence = SendSequence++,

                        LaserCount = CurrentLaserCount,
                        Transform = worldToLocal,
                        Points = Points,
                    });
                });
            }
        }

        void OnValidate()
        {
            if (TemplateIndex != 0)
            {
                var values = Simulator.Sensors.LidarSensor.Template.Templates[TemplateIndex];
                if (LaserCount != values.LaserCount ||
                    MinDistance != values.MinDistance ||
                    MaxDistance != values.MaxDistance ||
                    RotationFrequency != values.RotationFrequency ||
                    MeasurementsPerRotation != values.MeasurementsPerRotation ||
                    FieldOfView != values.FieldOfView ||
                    CenterAngle != values.CenterAngle ||
                    !Enumerable.SequenceEqual(VerticalRayAngles, values.VerticalRayAngles))
                {
                    TemplateIndex = 0;
                }
            }
        }
    }
}
