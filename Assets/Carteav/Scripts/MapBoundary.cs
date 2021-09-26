using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MapBoundary : MonoBehaviour
{
    public enum BoundaryType
    {
        MainArea,
        RestrictedArea
    }
    public Vector3 BoundaryOffset;
    public PolygonCollider2D PolygonCollider;
    public MeshFilter MeshFilter;
    public MeshRenderer Renderer;
    public BoundaryType Type;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
