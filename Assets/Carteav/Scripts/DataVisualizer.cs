using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carteav
{
    public class DataVisualizer : MonoBehaviour
    {
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
        private LineRenderer pathRenderer;
        [SerializeField]
        private Material waypointsMaterial;
        private Vector3 LineRendererPositionOffset = new Vector3(0.0f, 0.1f, 0.0f);
        

        public void VisualizePath(CarteavControlSensor.CartPath path, Vector3 offset)
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
                if (Physics.Raycast(point + raycastPositionOffset, raycastDirection, out hit, float.MaxValue, LayerMask.GetMask("Default")))
                {
                }
                else
                {
                    Physics.Raycast(point - raycastPositionOffset, -raycastDirection, out hit, float.MaxValue, LayerMask.GetMask("Default"));
                }

                point.y = hit.point.y + LineRendererPositionOffset.y;
                //Debug.Log($"Setting point:{point}, original:{waypointNode.Point}, hit:{hit.point}, index:{index + 1}");
                PathRenderer.SetPosition(index + 1, point);
                index++;
            }
        }
    }
}