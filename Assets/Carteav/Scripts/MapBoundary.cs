using System.Collections.Generic;
using System.Linq;
using Carteav.Messages;
using UnityEngine;

namespace Carteav
{
    public class MapBoundary : MonoBehaviour
    {
        public enum BoundaryType
        {
            MainArea,
            RestrictedArea
        }
        [field: SerializeField] public BoundaryType Type { get; private set; }
        [SerializeField] private  Vector3 BoundaryOffset;
        [SerializeField] private MeshFilter MeshFilter;
        [SerializeField] private  MeshRenderer Renderer;
        [SerializeField] private  PolygonCollider2D PolygonCollider;
        [field: SerializeField] public MeshCollider MeshCollider { get; private set; }
        [field: SerializeField] public  MeshCollider MeshEdgeCollider { get; private set; }
        
        private bool is2DMode;

        
        public void Setup(Polygon polygon, bool Is2DMode, bool extrudePolygon, Transform parent = null, 
            string boundaryName = null, Vector3 position = default, Quaternion rotation = default)
        {
            MapBoundary boundary = this;
            Transform boundaryTransform = boundary.transform;
            boundaryTransform.parent = parent;
            if (Type == BoundaryType.MainArea)
            {
                boundaryTransform.position = position;
            }
            else if (Type == BoundaryType.RestrictedArea)
            {
                boundaryTransform.localPosition = position;
            }
            boundaryTransform.rotation = rotation;
            var points3dList = polygon.Points.ConvertAll(vec3 => vec3);
            var points2d = points3dList.ConvertAll(vec3 => new Vector2(vec3.x, vec3.z)).ToArray();
            var points3d = points3dList.ToArray();
            boundary.name = boundaryName ?? boundary.Type.ToString();
            boundary.MeshFilter.mesh = CreatePolygonMesh(points3d, points2d);
            boundary.is2DMode = Is2DMode;
            
            if (Is2DMode)
            {
                boundary.PolygonCollider.points = points2d;
                if (boundary.Type == MapBoundary.BoundaryType.MainArea)
                {
                    EdgeCollider2D edgeCollider = boundary.GetComponent<EdgeCollider2D>();
                    edgeCollider.points = points2d;
                }
            }
            else
            {
                if (extrudePolygon)
                {
                    if (Type == BoundaryType.MainArea)
                    {
                        MeshCollider[] meshColliders = GetComponents<MeshCollider>();
                        boundary.MeshCollider = meshColliders[0];
                        boundary.MeshEdgeCollider = meshColliders[1];
                        boundary.MeshEdgeCollider.sharedMesh = CreatePolygonMesh3D(points3d, points2d, extrudePolygon, true);
                    }

                    boundary.MeshCollider.sharedMesh = CreatePolygonMesh3D(points3d, points2d, extrudePolygon);
                }
                else
                {
                    boundary.MeshCollider.sharedMesh = boundary.MeshFilter.mesh;
                }
            }
        }

        
        public void Dispose()
        {
            Destroy(MeshFilter.mesh);
            if (!is2DMode)
            {
                if (MeshCollider != null && MeshCollider.sharedMesh != null)
                {
                    Destroy(MeshCollider.sharedMesh);
                }
                if (MeshEdgeCollider != null && MeshEdgeCollider.sharedMesh != null)
                {
                    Destroy(MeshEdgeCollider.sharedMesh);
                }
            }
        }

        
        public void SetVisible(bool visibile)
        {
            Renderer.enabled = visibile;
        }


        private Mesh CreatePolygonMesh(Vector3[] points3d, Vector2[] points2d)
        {
            Mesh mesh = new Mesh();
            Triangulator triangulator = new Triangulator(points2d);
            mesh.vertices = points3d;
            mesh.triangles = triangulator.Triangulate().Reverse().ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        
        /// <summary>
        /// Creates a 3 dimensional mesh from a given points array representing a polygon.
        /// The mesh can be either flat representing the simple polygon in 3D space, or it can be
        /// 2 layered consisting of two duplicate polygons with a height difference.
        /// </summary>
        /// <param name="points3d"></param>
        /// <param name="points2d"></param>
        /// <param name="meshEdge">If true only mesh bounding edges will be used to create sort of a wall/fence mesh</param>
        /// /// <param name="extudePolygon">If true the 3D mesh created will be a two duplicates of the same
        /// polygon with a  height difference. If false only a single layer flat polyon 3D mesh will be created.</param>
        /// <returns></returns>
        private Mesh CreatePolygonMesh3D(Vector3[] points3d, Vector2[] points2d, bool extrudePolygon = true,
            bool meshEdge = false)
        {
            Mesh mesh = new Mesh();
            Triangulator triangulator = new Triangulator(points2d);
            var polygonTriangles = triangulator.Triangulate().Reverse().ToArray();


            Vector3 height = Vector3.zero;
            height.y = 1;

            var verts = new List<Vector3>(points3d);
            List<int> triangles = new List<int>(polygonTriangles);
            int vertexAmount = points3d.Length;

            if (extrudePolygon)
            {
                // duplicate the polygon vertices on a different height
                for (int i = 0; i < points3d.Length; i++)
                {
                    var vertex = points3d[i];
                    verts.Add(vertex - height);
                }


                // duplicate the original polygon's triangles only this time for the duplicated polygon's vertices
                for (int i = 0; i < mesh.triangles.Length; i++)
                {
                    triangles.Add(polygonTriangles[i] + vertexAmount);
                }

                List<(int, int)> boundaryEdges = new List<(int, int)>();
                // prepare edges connecting upper and lower polygon
                for (int i = 0; i < polygonTriangles.Length / 3; i++)
                {
                    boundaryEdges.Add((polygonTriangles[i * 3], polygonTriangles[i * 3 + 1]));
                    boundaryEdges.Add((polygonTriangles[i * 3 + 1], polygonTriangles[i * 3 + 2]));
                    boundaryEdges.Add((polygonTriangles[i * 3 + 2], polygonTriangles[i * 3]));
                }

                // remove non-bounding edges - for those that have the opposite edge present, remove both
                for (int i = 0; i < boundaryEdges.Count; i++)
                {
                    (int edgeA, int edgeB) = boundaryEdges[i];
                    if (boundaryEdges.Contains((edgeB, edgeA)) && meshEdge)
                    {
                        boundaryEdges.Remove(boundaryEdges[i]);
                        boundaryEdges.Remove((edgeB, edgeA));
                    }
                }

                // add triangles out of the bounding edges connecting upper and lower polygon
                for (int i = 0; i < boundaryEdges.Count; i++)
                {
                    (int edgeA, int edgeB) = boundaryEdges[i];
                    AddHeightTriangle(triangles, edgeA, edgeB, vertexAmount);
                }
            }

            mesh.vertices = verts.ToArray();
            mesh.triangles = triangles.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }


        private void AddHeightTriangle(List<int> triangles, int A, int B, int vertexAmount)
        {
            triangles.Add(A);
            triangles.Add(B);
            triangles.Add(B + vertexAmount);

            triangles.Add(B + vertexAmount);
            triangles.Add(A + vertexAmount);
            triangles.Add(A);
        }
    }
}