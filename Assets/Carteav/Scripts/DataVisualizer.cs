using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Simulator.ScenarioEditor.Utilities;
using Simulator.Utilities;
using UnityEngine;

namespace Carteav
{
    public class DataVisualizer : MonoBehaviour
    {

        [SerializeField] private MapBoundary boundaryPrefab;
        [SerializeField] private MapBoundary boundaryHolePrefab;
        [SerializeField] private Transform boundaryOrientationTransform;
        [SerializeField] private PrefabsPools pools;
        private List<MapBoundary> boundariesInUse = new List<MapBoundary>();
        private void Awake()
        {
            var test = new SiteBoundaries()
            {
                MultiPolygons = new List<SingleSiteBoundry>()
                {
                    new SingleSiteBoundry()
                    {
                        Polygons = new List<Polygon>()
                        {
                            new Polygon()
                            {
                                Points = new List<Vector3>()
                                {
                                    new Vector3(0, 0, 0),
                                    new Vector3(0, 1, 0),
                                    new Vector3(1, 1, 0),
                                    new Vector3(1, 0, 0)
                                }
                            }
                        }
                    }
                }
            };
            VisualizeBoundaries(test, 1f);
        }

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
        [SerializeField] private Material waypointsMaterial;
        private Vector3 LineRendererPositionOffset = new Vector3(0.0f, 0.1f, 0.0f);


        public void VisualizePath(CartPath path, Vector3 offset)
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

        
        
        
        public void VisualizeBoundaries(SiteBoundaries boundaries, float scale)
        {
            boundariesInUse.ForEach(boundary => pools.ReturnInstance(boundary.gameObject));
            boundariesInUse.Clear();
            var mainAreaPolygon = boundaries.MultiPolygons[0].Polygons[0];
            var offset = new Vector3();
            // mainAreaPolygon.Points.ForEach(p => offset += p);
            // offset /= mainAreaPolygon.Points.Count;
            
            MapBoundary mainArea = pools.GetInstance(boundaryPrefab.gameObject).GetComponent<MapBoundary>();
            InitBoundary(mainArea, mainAreaPolygon, offset, scale);
            var mainBoundaryTransform = mainArea.transform;
            mainBoundaryTransform.position = boundaryOrientationTransform.position;
            mainBoundaryTransform.localScale = boundaryOrientationTransform.localScale;
            mainBoundaryTransform.rotation = boundaryOrientationTransform.rotation;
            
            List<Polygon> holes = boundaries.MultiPolygons[0].Polygons.GetRange(1, boundaries.MultiPolygons[0].Polygons.Count - 1);
            foreach (var hole in holes)
            {
                var holeObj = pools.GetInstance(boundaryHolePrefab.gameObject).GetComponent<MapBoundary>();
                InitBoundary(holeObj, hole, offset, scale);
                var holeTransform = holeObj.transform;
                holeTransform.parent = mainBoundaryTransform;
                holeTransform.localPosition = new Vector3(0, 0, -0.01f);
                holeTransform.localRotation = Quaternion.identity;
                holeTransform.localScale = Vector3.one;
            }
            
        }

        private void InitBoundary(MapBoundary boundary, Polygon polygon, Vector3 offset, float scale)
        {
            var points3d = polygon.Points.ConvertAll(vec3 => (vec3 + offset) * scale);
            boundary.PolygonCollider.points = points3d.ConvertAll(vec3 => new Vector2(vec3.x, vec3.z)).ToArray();
            var mesh = CreatePolygonMesh(points3d);
            boundary.MeshFilter.mesh = mesh;
            boundary.BoundaryOffset = offset;
            boundariesInUse.Add(boundary);
        }
        
        private static Mesh CreatePolygonMesh(List<Vector3> points3d)
        {
            Mesh mesh = new Mesh(); //CreateMesh(points.ConvertAll(vec3 => (vec3 - com) * scale),0);
            //polygonCollider.CreateMesh(true, true);
            //mesh.vertices = points.ConvertAll(vec3 => (vec3-com)*scale).ToArray();
            Triangulator triangulator = new Triangulator(points3d.ConvertAll(vec3 => new Vector2(vec3.x,vec3.z)).ToArray());
            mesh.vertices = points3d.ToArray();
            mesh.triangles = triangulator.Triangulate().Reverse().ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
            
        }


        public class Triangulator
        {
            private List<Vector2> m_points = new List<Vector2>();

            public Triangulator(Vector2[] points)
            {
                m_points = new List<Vector2>(points);
            }

            public int[] Triangulate()
            {
                List<int> indices = new List<int>();

                int n = m_points.Count;
                if (n < 3)
                    return indices.ToArray();

                int[] V = new int[n];
                if (Area() > 0)
                {
                    for (int v = 0; v < n; v++)
                        V[v] = v;
                }
                else
                {
                    for (int v = 0; v < n; v++)
                        V[v] = (n - 1) - v;
                }

                int nv = n;
                int count = 2 * nv;
                for (int v = nv - 1; nv > 2;)
                {
                    if ((count--) <= 0)
                        return indices.ToArray();

                    int u = v;
                    if (nv <= u)
                        u = 0;
                    v = u + 1;
                    if (nv <= v)
                        v = 0;
                    int w = v + 1;
                    if (nv <= w)
                        w = 0;

                    if (Snip(u, v, w, nv, V))
                    {
                        int a, b, c, s, t;
                        a = V[u];
                        b = V[v];
                        c = V[w];
                        indices.Add(a);
                        indices.Add(b);
                        indices.Add(c);
                        for (s = v, t = v + 1; t < nv; s++, t++)
                            V[s] = V[t];
                        nv--;
                        count = 2 * nv;
                    }
                }

                indices.Reverse();
                return indices.ToArray();
            }

            private float Area()
            {
                int n = m_points.Count;
                float A = 0.0f;
                for (int p = n - 1, q = 0; q < n; p = q++)
                {
                    Vector2 pval = m_points[p];
                    Vector2 qval = m_points[q];
                    A += pval.x * qval.y - qval.x * pval.y;
                }

                return (A * 0.5f);
            }

            private bool Snip(int u, int v, int w, int n, int[] V)
            {
                int p;
                Vector2 A = m_points[V[u]];
                Vector2 B = m_points[V[v]];
                Vector2 C = m_points[V[w]];
                if (Mathf.Epsilon > (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x))))
                    return false;
                for (p = 0; p < n; p++)
                {
                    if ((p == u) || (p == v) || (p == w))
                        continue;
                    Vector2 P = m_points[V[p]];
                    if (InsideTriangle(A, B, C, P))
                        return false;
                }

                return true;
            }

            private bool InsideTriangle(Vector2 A, Vector2 B, Vector2 C, Vector2 P)
            {
                float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
                float cCROSSap, bCROSScp, aCROSSbp;

                ax = C.x - B.x;
                ay = C.y - B.y;
                bx = A.x - C.x;
                by = A.y - C.y;
                cx = B.x - A.x;
                cy = B.y - A.y;
                apx = P.x - A.x;
                apy = P.y - A.y;
                bpx = P.x - B.x;
                bpy = P.y - B.y;
                cpx = P.x - C.x;
                cpy = P.y - C.y;

                aCROSSbp = ax * bpy - ay * bpx;
                cCROSSap = cx * apy - cy * apx;
                bCROSScp = bx * cpy - by * cpx;

                return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
            }
        }
    }
}