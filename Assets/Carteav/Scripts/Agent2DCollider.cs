using Carteav;
using Carteav.Messages;
using UnityEngine;

public class Agent2DCollider : MonoBehaviour
{
    [field: SerializeField] public PolygonCollider2D AgentCollider2D { get; set; }
    [SerializeField] private DataHandler dataHandler;
    [SerializeField] private Rigidbody2D rigidbody2D;
    private Transform agentTransform;
    private bool insideParmittedArea;

    private void Start()
    {
        agentTransform = transform;
    }

    private void OnTriggerStay2D(Collider2D other) 
    { 
        //Debug.Log($"OnTriggerStay2D this:{gameObject.name}  other:{other.transform.name}"); 
        var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>(); 
        if (mapBoundary != null && mapBoundary.Type == MapBoundary.BoundaryType.MainArea)
        {
            insideParmittedArea = true;
        } 
    } 
 
    private void OnTriggerEnter2D(Collider2D other) 
    { 
        Debug.Log($"OnTriggerEnter2D this:{gameObject.name}  other:{other.transform.name}"); 
        var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>(); 
        if (mapBoundary != null ) 
        {
            switch (mapBoundary.Type)
            {
                case MapBoundary.BoundaryType.MainArea:
                    if (insideParmittedArea && other is EdgeCollider2D)
                    {
                        Debug.Log("main area exited");
                        dataHandler.SendBoundaryCross(new BoundaryCross()
                        {
                            ObjectName = other.gameObject.name,
                            Position = agentTransform.position,
                            Velocity = rigidbody2D.velocity,
                            YawAngle = agentTransform.eulerAngles.x,
                            Time = SimulatorManager.Instance.CurrentTime
                        });
                    }
                    break;
                case MapBoundary.BoundaryType.RestrictedArea:
                    Debug.Log("restricted area entered");
                    dataHandler.SendBoundaryCross(new BoundaryCross()
                    {
                        ObjectName = other.gameObject.name,
                        Position = agentTransform.position,
                        Velocity = rigidbody2D.velocity,
                        YawAngle = agentTransform.eulerAngles.x,
                        Time = SimulatorManager.Instance.CurrentTime
                    });
                    break;
            }
        } 
    }

    private void OnTriggerExit2D(Collider2D other)
    {
        Debug.Log($"OnTriggerExit2D this:{gameObject.name}  other:{other.transform.name}");
        var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>();
        if (mapBoundary != null && mapBoundary.Type == MapBoundary.BoundaryType.MainArea)
        {
            if (other is PolygonCollider2D)
            {
                insideParmittedArea = false;
            }
            
        }
    }
    
    private void OnCollisionEnter2D(Collision2D other) 
    { 
        //Debug.Log("OnCollisionEnter2D"); 
    } 
 
         
    private void OnCollisionStay2D(Collision2D other) 
    { 
        //Debug.Log("OnCollisionStay2D"); 
    } 
         
    private void OnCollisionExit2D(Collision2D other) 
    { 
        //Debug.Log("OnCollisionExit2D"); 
    } 

}
