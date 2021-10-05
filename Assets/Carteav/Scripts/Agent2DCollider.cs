using System;
using System.Collections;
using System.Collections.Generic;
using Carteav;
using UnityEngine;

public class Agent2DCollider : MonoBehaviour
{
    [field: SerializeField] public PolygonCollider2D AgentCollider2D { get; set; }
    [SerializeField] private DataHandler dataHandler;

    private bool insideParmittedArea;

    

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

                        });
                    }
                    break;
                case MapBoundary.BoundaryType.RestrictedArea:
                    Debug.Log("restricted area entered");
                    dataHandler.SendBoundaryCross(new BoundaryCross()
                    {

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
