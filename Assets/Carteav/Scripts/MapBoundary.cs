using System.Collections.Generic;
using System.Linq;
using Carteav.Messages;
using UnityEngine;

namespace Carteav
{
    /// <summary>
    /// Allocates colliders according to selected method. For 2D polygonCollider and edgeCollider and for 3D
    /// MeshCollider and MeshEdgeCollider. The polygon collider to check if the agent was already inside when touching
    /// the edge collider.
    /// </summary>
    public class MapBoundary : MonoBehaviour
    {
        public enum BoundaryType
        {
            MainArea,
            RestrictedArea
        }

        [field: SerializeField] public BoundaryType Type { get; private set; }
        public MeshCollider MeshPolygonCollider { get; private set; }
        public MeshCollider MeshEdgeCollider { get; private set; }

        [SerializeField] private MeshFilter meshFilter;
        [SerializeField] private MeshRenderer meshRenderer;
        [SerializeField] private PolygonCollider2D polygonCollider;
        [SerializeField] private EdgeCollider2D edgeCollider;
        private bool is2DMode;


        public void Setup(Polygon polygon, bool Is2DMode, Transform parent = null,
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
            boundary.meshFilter.mesh = CreatePolygonMesh(points3d, points2d);
            boundary.is2DMode = Is2DMode;

            if (Is2DMode)
            {
                boundary.polygonCollider.points = points2d;
                if (boundary.Type == MapBoundary.BoundaryType.MainArea)
                {
                    edgeCollider.points = points2d;
                }
            }
            else
            {
                bool extudePolygonToHaveHeight = true;
                if (extudePolygonToHaveHeight)
                {
                    MeshCollider[] meshColliders = GetComponents<MeshCollider>();
                    boundary.MeshPolygonCollider = meshColliders[0];
                    boundary.MeshPolygonCollider.sharedMesh = CreatePolygonMesh3D(points3d, points2d);
                    if (Type == BoundaryType.MainArea)
                    {
                        boundary.MeshEdgeCollider = meshColliders[1];
                        boundary.MeshEdgeCollider.sharedMesh = CreatePolygonMesh3D(points3d, points2d, true);
                    }
                }
                else
                {
                    boundary.MeshPolygonCollider.sharedMesh = boundary.meshFilter.mesh;
                }
            }
        }


        public void Dispose()
        {
            Destroy(meshFilter.mesh);
            if (!is2DMode)
            {
                if (MeshPolygonCollider != null && MeshPolygonCollider.sharedMesh != null)
                {
                    Destroy(MeshPolygonCollider.sharedMesh);
                }

                if (MeshEdgeCollider != null && MeshEdgeCollider.sharedMesh != null)
                {
                    Destroy(MeshEdgeCollider.sharedMesh);
                }
            }
        }


        public void SetVisible(bool visibile)
        {
            meshRenderer.enabled = visibile;
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
        private Mesh CreatePolygonMesh3D(Vector3[] points3d, Vector2[] points2d, bool meshEdge = false, float height = 2f)
        {
            Mesh mesh = new Mesh();
            Triangulator triangulator = new Triangulator(points2d);
            var polygonTriangles = triangulator.Triangulate().Reverse().ToArray();


            Vector3 heightOffset = Vector3.zero;
            heightOffset.y = height;

            var verts = new List<Vector3>(points3d);
            List<int> triangles = new List<int>(polygonTriangles);
            int vertexAmount = points3d.Length;


            // duplicate the polygon vertices on a different height
            for (int i = 0; i < points3d.Length; i++)
            {
                var vertex = points3d[i];
                verts.Add(vertex - heightOffset / 2);
                //points3d[i] = vertex + height / 2;
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