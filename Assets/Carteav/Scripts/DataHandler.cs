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
        [SerializeField] private Agent2DCollider agentCollider2D;
        [SerializeField] private MapBoundary boundaryPrefab;
        [SerializeField] private MapBoundary boundaryHolePrefab;
        [SerializeField] private MapBoundary boundary3DPrefab;
        [SerializeField] private MapBoundary boundaryHole3DPrefab;
        [SerializeField] private Transform boundaryOrientation;
        [SerializeField] private PrefabsPools pools;
        [SerializeField] private Material waypointsMaterial;


        public bool Is2DMode
        {
            get { return is2DMode; }
            set
            {
                if (is2DMode != value && currentBoundaries != null && boundariesInUse.Count > 0)
                {
                    Dispose();
                    is2DMode = value;
                    HandleBoundaries(currentBoundaries);
                    agentCollider2D.gameObject.SetActive(is2DMode);
                    return;
                }

                is2DMode = value;
            }
        }
        private bool is2DMode;
        private List<MapBoundary> boundariesInUse = new List<MapBoundary>();
        private Publisher<BoundaryCross> publishBoundaryCross;
        private Transform agentTransform;
        private LineRenderer pathRenderer;
        private Vector3 LineRendererPositionOffset = new Vector3(0.0f, 0.1f, 0.0f);
        private SiteBoundaries currentBoundaries;

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



        public void Setup(Publisher<BoundaryCross> publisherBoundaryCross, Transform agentTransform, bool is2DMode)
        {
            this.agentTransform = agentTransform;
            publishBoundaryCross = publisherBoundaryCross;
            agentCollider2D.Setup(agentTransform);
            agentCollider2D.gameObject.SetActive(is2DMode);
            Is2DMode = is2DMode;
        }


        public void SendBoundaryCross(BoundaryCross boundaryCross)
        {
            publishBoundaryCross(boundaryCross);
        }


        public void Update2DPosition(Vector2 position)
        {
            agentCollider2D.transform.localPosition = -position;
        }


        public void ToggleData(bool isShown)
        {
            foreach (var mapBoundary in boundariesInUse)
            {
                mapBoundary.SetVisible(isShown);
            }

            PathRenderer.enabled = isShown;
        }


        public void Dispose()
        {
            boundariesInUse.ForEach(boundary =>
            {
                boundary.Dispose();
                
                pools.ReturnInstance(boundary.gameObject);
            });
            boundariesInUse.Clear();
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
        

        public void HandleBoundaries(SiteBoundaries boundaries)
        {
            currentBoundaries = boundaries;
            Dispose();
            
            GameObject permittedAreaPrefab = Is2DMode ? boundaryPrefab.gameObject : boundary3DPrefab.gameObject;
            GameObject restrictedAreaPrefab =
                Is2DMode ? boundaryHolePrefab.gameObject : boundaryHole3DPrefab.gameObject;
            var mainAreaPolygon = boundaries.MultiPolygons[0].Polygons[0];

            MapBoundary mainArea = pools.GetInstance(permittedAreaPrefab).GetComponent<MapBoundary>();
            mainArea.Setup(mainAreaPolygon, Is2DMode, boundaryOrientation.parent, 
                mainArea.Type.ToString(), boundaryOrientation.position, boundaryOrientation.rotation);
            boundariesInUse.Add(mainArea);

            if (Is2DMode)
            {
                agentCollider2D.transform.parent = mainArea.transform;
            }

            List<Polygon> holes = boundaries.MultiPolygons[0].Polygons
                .GetRange(1, boundaries.MultiPolygons[0].Polygons.Count - 1);
            Vector3 restrictedOffset = new Vector3(0, 0, -0.01f);
            for (int i = 0; i < holes.Count; i++)
            {
                var hole = holes[i];
                var restrictedArea = pools.GetInstance(restrictedAreaPrefab).GetComponent<MapBoundary>();
                restrictedArea.Setup(hole, Is2DMode, mainArea.transform, 
                    $"{restrictedArea.Type} - {i}", restrictedOffset);
                boundariesInUse.Add(restrictedArea);
            }
        }
    }
}