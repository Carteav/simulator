using System.Collections.Generic;
using System.Linq;
using Carteav.Messages;
using Simulator.Bridge;
using Simulator.ScenarioEditor.Utilities;
using UnityEngine;

namespace Carteav
{
    public class DataHandler : MonoBehaviour
    {
        [SerializeField] private Agent2DCollider agentCollider;
        [SerializeField] private MapBoundary boundaryPrefab;
        [SerializeField] private MapBoundary boundaryHolePrefab;
        [SerializeField] private Transform boundaryOrientationTransform;
        [SerializeField] private PrefabsPools pools;
        [SerializeField] private bool Is3DMode = false;
        [SerializeField] private Material waypointsMaterial;
        
        private List<MapBoundary> boundariesInUse = new List<MapBoundary>();
        private Publisher<BoundaryCross> publishBoundaryCross;
        private GameObject agent;
        private LineRenderer pathRenderer;
        private Vector3 LineRendererPositionOffset = new Vector3(0.0f, 0.1f, 0.0f);

        public LineRenderer PathRenderer
        {
            get
            {
                if (pathRenderer != null) return pathRenderer;

                pathRenderer = gameObject.GetComponent<LineRenderer>();
                if (pathRenderer == null)
                {
                    pathRenderer = gameObject.AddComponent<LineRenderer>();
                    pathRenderer.material = waypointsMaterial;
                    pathRenderer.useWorldSpace = false;
                    pathRenderer.positionCount = 1;
                    pathRenderer.SetPosition(0, LineRendererPositionOffset);
                    pathRenderer.sortingLayerName = "Ignore Raycast";
                    pathRenderer.widthMultiplier = 0.1f;
                    pathRenderer.generateLightingData = false;
                    pathRenderer.textureMode = LineTextureMode.Tile;
                }

                return pathRenderer;
            }
        }

        
        public void HandlePath(CartPath path, Vector3 offset)
        {
            PathRenderer.positionCount = path.Points.Count + 2;
            //Debug.Log($"Setting {path.Points.Count} points");
            int index = 0;
            Vector3 raycastPositionOffset = new Vector3(0, 1000, 0);
            Vector3 raycastDirection = new Vector3(0, -1, 0);
            RaycastHit hit;
            foreach (var waypointNode in path.Points)
            {
                Vector3 point = waypointNode.Point + offset;
                point.y = 0;
                if (Physics.Raycast(point + raycastPositionOffset, raycastDirection, out hit, float.MaxValue,
                    LayerMask.GetMask("Default")))
                {
                }
                else
                {
                    Physics.Raycast(point - raycastPositionOffset, -raycastDirection, out hit, float.MaxValue,
                        LayerMask.GetMask("Default"));
                }

                point.y = hit.point.y + LineRendererPositionOffset.y;
                //Debug.Log($"Setting point:{point}, original:{waypointNode.Point}, hit:{hit.point}, index:{index + 1}");
                PathRenderer.SetPosition(index + 1, point);
                index++;
            }
        }

        
        public void Setup(Publisher<BoundaryCross> publisherBoundaryCross, GameObject agentObject)
        {
            agent = agentObject;
            publishBoundaryCross = publisherBoundaryCross;
        }
        
        
        public void SendBoundaryCross(BoundaryCross boundaryCross)
        {
            publishBoundaryCross(boundaryCross);
        }
        

        public void Update2DPosition(Vector2 position)
        {
            var agentTransform = agentCollider.transform;
            agentTransform.localPosition = -position;
        }
        
        
        public void ToggleBoundaries(bool isShown)
        {
            foreach (var mapBoundary in boundariesInUse)
            {
                mapBoundary.Renderer.enabled = isShown;
            }
        }

        
        public void Dispose()
        {
            boundariesInUse.ForEach(boundary =>
            {
                Destroy(boundary.MeshFilter.mesh);
                pools.ReturnInstance(boundary.gameObject);
            });
        }
        
        
        public void HandleBoundaries(SiteBoundaries boundaries)
        {
            Dispose();
            boundariesInUse.Clear();
            var mainAreaPolygon = boundaries.MultiPolygons[0].Polygons[0];
           
            
            MapBoundary mainArea = pools.GetInstance(boundaryPrefab.gameObject).GetComponent<MapBoundary>();
            InitBoundary(mainArea, mainAreaPolygon);
            var mainBoundaryTransform = mainArea.transform;
            mainBoundaryTransform.position = boundaryOrientationTransform.position;
            mainBoundaryTransform.localScale = boundaryOrientationTransform.localScale;
            mainBoundaryTransform.rotation = boundaryOrientationTransform.rotation;
            mainBoundaryTransform.parent = transform;
            if (!Is3DMode)
            {
                agentCollider.transform.parent = mainBoundaryTransform;
                agentCollider.transform.rotation = Quaternion.identity;
                var meshCollider = agent.transform.GetChild(0).GetComponentInChildren<Collider>();
                var agentBounds = meshCollider.bounds;
                Vector2[] points = new List<Vector2>()
                {
                    new Vector2() { x = agentBounds.max.x, y = agentBounds.max.y},
                    new Vector2() { x = agentBounds.max.x, y = agentBounds.min.y},
                    new Vector2() { x = agentBounds.min.x, y = agentBounds.min.y},
                    new Vector2() { x = agentBounds.min.x, y = agentBounds.max.y},
                }.ToArray();
                //agentCollider.AgentCollider2D.points = points;
            }
            
            
            List<Polygon> holes = boundaries.MultiPolygons[0].Polygons.GetRange(1, boundaries.MultiPolygons[0].Polygons.Count - 1);
            for (int i = 0; i < holes.Count; i++)
            {
                var hole = holes[i];
                var holeObj = pools.GetInstance(boundaryHolePrefab.gameObject).GetComponent<MapBoundary>();
                InitBoundary(holeObj, hole);
                var holeTransform = holeObj.transform;
                holeObj.name = $"{holeObj.Type} - {i}";
                holeTransform.parent = mainBoundaryTransform;
                holeTransform.localPosition = new Vector3(0, 0, -0.01f);
                holeTransform.localRotation = Quaternion.identity;
                holeTransform.localScale = Vector3.one;
            }
        }

        
        private void InitBoundary(MapBoundary boundary, Polygon polygon)
        {
            boundary.name = boundary.Type.ToString();
            var points3d = polygon.Points.ConvertAll(vec3 => vec3);
            var points2d = points3d.ConvertAll(vec3 => new Vector2(vec3.x,vec3.z)).ToArray();
            boundary.PolygonCollider.points = points2d;
            var mesh = CreatePolygonMesh(points3d, points2d);
            boundary.MeshFilter.mesh = mesh;
            //boundary.BoundaryOffset = polygon.Offset;
            boundariesInUse.Add(boundary);

            if (boundary.Type == MapBoundary.BoundaryType.MainArea)
            {
                EdgeCollider2D edgeCollider = boundary.GetComponent<EdgeCollider2D>();
                edgeCollider.points = points2d;
            }
        }
        
        
        private static Mesh CreatePolygonMesh(List<Vector3> points3d, Vector2[] points2d)
        {
            Mesh mesh = new Mesh(); 
            Triangulator triangulator = new Triangulator(points2d);
            mesh.vertices = points3d.ToArray();
            mesh.triangles = triangulator.Triangulate().Reverse().ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}