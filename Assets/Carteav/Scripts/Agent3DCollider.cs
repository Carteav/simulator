using System.Collections;
using System.Collections.Generic;
using Carteav;
using Carteav.Messages;
using Simulator.Bridge;
using UnityEngine;

public class Agent3DCollider : MonoBehaviour
{
    [SerializeField] private MeshCollider agentMeshCollider;
    private Publisher<BoundaryCross> boundaryCrossPublisher;
    private Rigidbody agentRigidBody;
    
    private bool insidePermittedArea = false;
    private Transform cartTransform;


    

    public void Setup(Rigidbody agentRigidBody, Publisher<BoundaryCross> boundaryCrossPublisher, Transform cartTransform)
    {
        this.agentRigidBody = agentRigidBody;
        this.boundaryCrossPublisher = boundaryCrossPublisher;
        this.cartTransform = cartTransform;
        
        Mesh mesh = null;
        var meshCollider = cartTransform.GetComponentInChildren<MeshCollider>();
        if (meshCollider != null)
        {
            mesh = meshCollider.sharedMesh;
        }
        else
        {
            var meshFilter = cartTransform.GetComponentInChildren<MeshFilter>();
            if (meshFilter != null)
            {
                mesh = meshFilter.mesh;
            }
        }
        agentMeshCollider.sharedMesh = mesh;
    }


    private void OnTriggerStay(Collider other)
    {
        //Debug.Log($"OnTriggerStay this:{gameObject.name}  other:{other.transform.name}");
        var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>();
        if (mapBoundary != null)
        {
            if (mapBoundary.Type == MapBoundary.BoundaryType.MainArea && other == mapBoundary.MeshPolygonCollider)
            {
                insidePermittedArea = true;
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        //Debug.Log($"OnTriggerEnter this:{gameObject.name}  other:{other.transform.name}");
        var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>();
        if (mapBoundary != null)
        {
            switch (mapBoundary.Type)
            {
                case MapBoundary.BoundaryType.MainArea:
                    if (insidePermittedArea && other == mapBoundary.MeshEdgeCollider)
                    {
                        Debug.Log("main area exited");
                        boundaryCrossPublisher(new BoundaryCross()
                        {
                            ObjectName = other.gameObject.name,
                            Position = cartTransform.position,
                            Velocity = agentRigidBody.velocity,
                            Time = SimulatorManager.Instance.CurrentTime
                        });
                    }

                    break;

                case MapBoundary.BoundaryType.RestrictedArea:
                    Debug.Log("restricted area entered");
                    boundaryCrossPublisher(new BoundaryCross()
                    {
                        ObjectName = other.gameObject.name,
                        Position = cartTransform.position,
                        Velocity = agentRigidBody.velocity,
                        Time = SimulatorManager.Instance.CurrentTime
                    });
                    break;
            }
        }
    }


    private void OnTriggerExit(Collider other)
    {
        //Debug.Log($"OnTriggerExit this:{gameObject.name}  other:{other.transform.parent.name}");
        var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>();
        if (mapBoundary != null && mapBoundary.Type == MapBoundary.BoundaryType.MainArea)
        {
            Debug.Log("main area exited");
            insidePermittedArea = false;
        }
    }
}