using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading.Tasks;
using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Utilities;
using UnityEngine;
using UnityEngine.Rendering;


namespace Simulator.Sensors
{
    /// <summary>
    /// CarteavLidarSensor is a custom lidar sensor.
    /// Instance fields are named with a "custom" prefix to differentiate from default implementation variables.
    /// </summary>
    //[SensorType("Lidar", new[] { typeof(PointCloudData) })]
    public class CarteavLidarSensor : CarteavLidarBase
    {
       
        public class LayoutData
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
        [SerializeField] private bool custom;
        [SerializeField] private string anglesFile;
        [SerializeField] private int angleLayouts;
        [SerializeField] private int anglesPerLayout;
        [SerializeField] private float timePerLayout;

        [SerializeField]
        private bool projectAngles;
        
            
        protected BaseLink baseLink;
        protected Dictionary<int, LayoutData> angleFileLayouts = new Dictionary<int, LayoutData>();
        
        private int customPoints = 100000;
        private int customSectors = 24;
        private bool lastFrameCustom;
        private float timeSinceLayoutChange;
        private int layoutIndex;
        
 
        protected override int Kernel => computeShader.FindKernel(Compensated ? "CarteavCubeComputeComp" : "CarteavCubeCompute");

        public override void Reset()
        {
            if (custom)
            {
                var maxPointsPerLayout = angleFileLayouts.Values.Max(layoutData => layoutData.MaxPointsPerSector);
                TotalPointCount = maxPointsPerLayout * customSectors;
            }
            else
            {
                DispatchThreads = new Vector3Int(HDRPUtilities.GetGroupSize(MeasurementsPerRotation, 8),
                    HDRPUtilities.GetGroupSize(LaserCount, 8), 1);
            }
            
            base.Reset();
        }

        

        public override void Update()
        {
            if (lastFrameCustom != custom)
            {
                lastFrameCustom = custom;
                Reset();
            }

            base.Update();
        }

        protected override void Initialize()
        {
            baseLink = transform.parent.GetComponentInChildren<BaseLink>();
            timeSinceLayoutChange = Time.time;
            GetTwoArraysFromFile(Application.dataPath + $"/Carteav/Resources/Angles/{anglesFile}");//LoadFileAngleData();
            base.Initialize();
        }

        protected override void Deinitialize()
        {
            foreach (var layoutData in angleFileLayouts.Values)
            {
                ReleaseCustomBuffers(layoutData);
            }
        }

      
        protected override bool SetComputeParams(CommandBuffer cmd, int kernel)
        {
            bool shouldRender = true;
            shouldRender = base.SetComputeParams(cmd, kernel);
            
            if (Time.time - timeSinceLayoutChange > timePerLayout && angleLayouts > 1)//
            {
                
                layoutIndex = (layoutIndex + 1) % angleLayouts;
            }
            
            // Set Custom Properties //
            cmd.SetComputeIntParam(cs, Properties.SectorsCustom, customSectors);
            cmd.SetComputeIntParam(cs, Properties.PointsCustom, angleFileLayouts[layoutIndex].MaxPointsPerSector);
            cmd.SetComputeBufferParam(cs, kernel, Properties.SizeCustom, angleFileLayouts[layoutIndex].CustomSizeBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.IndexCustom, angleFileLayouts[layoutIndex].CustomIndexBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.PitchCustom, angleFileLayouts[layoutIndex].CustomPitchBuffer);
            cmd.SetComputeBufferParam(cs, kernel, Properties.YawCustom, angleFileLayouts[layoutIndex].CustomYawBuffer);
            cmd.SetComputeIntParam(cs, Properties.ProjectAngles, projectAngles ? 1 : 0);
            //

            int yDimension = angleFileLayouts[layoutIndex].CustomSize.Max() + 1;
            if (yDimension > 0)
            {
                DispatchThreads = new Vector3Int(HDRPUtilities.GetGroupSize(customSectors,8), HDRPUtilities.GetGroupSize(yDimension,8), 1);
            }
            else
            {
                return false;
            }
            return shouldRender;
        }


        protected override void SendMessage()
        {
            if (!(Bridge is { Status: Status.Connected }))
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
                Publish(new PointCloudData()
                {
                    Name = Name,
                    Frame = Frame,
                    Time = SimulatorManager.Instance.CurrentTime,
                    Sequence = Sequence++,
                    LaserCount = CurrentLaserCount,
                    Transform = pointsTransform,
                    Points = Points,
                    PointCount = Points.Length
                });
            });
        }
        
        
        /*private void LoadFileAngleData()
        {
            string path = Application.dataPath;
            //Copy the LIDAR angles file
            try
            {
                // Will not overwrite if the destination file already exists.
                System.IO.File.Copy(path + $"/Carteav/Resources/Angles/{anglesFile}", path + $"/../simulator_Data/{anglesFile}", true);
            }
            // Catch exception if the file was already copied.
            catch (IOException copyError)
            {
                Console.WriteLine(copyError.Message);
            }

            //Read LIDAR angles file
            GetTwoArraysFromFile(path + $"/Carteav/Resources/Angles/{anglesFile}");
        }*/

        private void ReleaseCustomBuffers(LayoutData layoutData)
        {
            if (layoutData.CustomSizeBuffer != null)
            {
                layoutData.CustomSizeBuffer.Release();
                layoutData.CustomSizeBuffer = null;
            }

            if (layoutData.CustomIndexBuffer != null)
            {
                layoutData.CustomIndexBuffer.Release();
                layoutData.CustomIndexBuffer = null;
            }

            if (layoutData.CustomYawBuffer != null)
            {
                layoutData.CustomYawBuffer.Release();
                layoutData.CustomYawBuffer = null;
            }

            if (layoutData.CustomPitchBuffer != null)
            {
                layoutData.CustomPitchBuffer.Release();
                layoutData.CustomPitchBuffer = null;
            }
        }


        public void GetTwoArraysFromFile(string filein)
        {
            //angleLayouts = 1;
            System.IO.StreamReader file = new System.IO.StreamReader(filein);
            for (int layout = 0; layout < angleLayouts; layout++)
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
                    String[] columns = line.Trim().Split(',');
                    float radiansA = float.Parse(columns[0], CultureInfo.InvariantCulture);
                    float degreesB = radiansA * 180f / Mathf.PI;
                    int sectorG = degreesB > 0 ? (int)(degreesB / 15f) : (int)(degreesB / 15f) - 1;
                    int positiveSectorH = sectorG < 0 ? sectorG + 24 : sectorG;
                    /* float degreesOffsetC = sectorG * 15 - degreesB + 90f + 7.5f; //(15 / 2)
                    float radianD_Result1 = degreesOffsetC * Mathf.PI / 180f;*/
                    float radianF_Result2 = float.Parse(columns[1], CultureInfo.InvariantCulture) ;//+ Mathf.PI
                    p1[positiveSectorH].Add(radiansA);//radianD_Result1
                    p2[positiveSectorH].Add(radianF_Result2);
                    lines[positiveSectorH]++;
                    lineCount++;
                    // Debug.Log($"Line[{lineCount++}] - A:{radiansA}, B:{degreesB}, C:{degreesOffsetC}, D:{radianD_Result1}, E:{radiansE}, F:{radianF_Result2}, G:{sectorG}, H:{positiveSectorH}.\nPass:{passIndex}");
                    if (angleLayouts > 1 && anglesPerLayout > 0 && lineCount > anglesPerLayout) break;
                }
                int maxPointsPerSector = lines.Max() + 1;//customPoints;// ?
               
                angleFileLayouts[layout] = new LayoutData
                {
                    P1 = p1, P2 = p2, 
                    Lines = lines, 
                    CustomYaw = new float[customSectors, maxPointsPerSector],
                    CustomPitch = new float[customSectors, maxPointsPerSector], 
                    CustomIndex = new int[customSectors],
                    CustomSize = new int[customSectors],
                    MaxPointsPerSector = maxPointsPerSector
                };

                ExtractAngles(angleFileLayouts[layout]);
            }
        }

        private void ExtractAngles(LayoutData layoutData)
        {
            int sum = 0;
            for (int i = 0; i < customSectors; i++)
            {
                for (int j = 0; j < layoutData.Lines[i]; j++) //layoutData.Lines[i]
                {
                    layoutData.CustomYaw[i, j] = layoutData.P1[i][j];
                    layoutData.CustomPitch[i, j] = layoutData.P2[i][j];
                }

                layoutData.CustomSize[i] = layoutData.Lines[i];
                layoutData.CustomIndex[i] = sum;
                sum += layoutData.CustomSize[i];
                //print("i= " + i + " " + size[i] + " " + index[i]);
            }
            
            layoutData.CustomSizeBuffer = new ComputeBuffer(customSectors, 4);
            layoutData.CustomIndexBuffer = new ComputeBuffer(customSectors, 4);
            layoutData.CustomYawBuffer = new ComputeBuffer(customSectors * layoutData.MaxPointsPerSector, 4);
            layoutData.CustomPitchBuffer = new ComputeBuffer(customSectors * layoutData.MaxPointsPerSector, 4);
            layoutData.CustomSizeBuffer.SetData(layoutData.CustomSize);
            layoutData.CustomIndexBuffer.SetData(layoutData.CustomIndex);
            layoutData.CustomYawBuffer.SetData(layoutData.CustomYaw);
            layoutData.CustomPitchBuffer.SetData(layoutData.CustomPitch);
            
        }
    }
}