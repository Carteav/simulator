using System.Collections.Generic;
using Carteav;
using Carteav.Messages;
using UnityEngine;

namespace Carteav
{


    public class Agent2DCollider : MonoBehaviour
    {
        [field: SerializeField] public PolygonCollider2D AgentCollider2D { get; set; }
        [SerializeField] private DataHandler dataHandler;
        [SerializeField] private Rigidbody2D rigidbody2D;
        private Transform agentTransform;
        private bool insideParmittedArea;


        public void Setup(Transform agentTransform)
        {
            this.agentTransform = agentTransform;
            AssignColliderPoints();
        }


        private void AssignColliderPoints()
        {
            var meshCollider = agentTransform.GetComponentInChildren<Collider>();
            var agentBounds = meshCollider.bounds;
            Vector2[] points = new List<Vector2>()
            {
                new Vector2() { x = agentBounds.max.x, y = agentBounds.max.y },
                new Vector2() { x = agentBounds.max.x, y = agentBounds.min.y },
                new Vector2() { x = agentBounds.min.x, y = agentBounds.min.y },
                new Vector2() { x = agentBounds.min.x, y = agentBounds.max.y },
            }.ToArray();
            //agentCollider.AgentCollider2D.points = points;
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
            //Debug.Log($"OnTriggerEnter2D this:{gameObject.name}  other:{other.transform.name}");
            var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>();
            if (mapBoundary != null)
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
                            Time = SimulatorManager.Instance.CurrentTime
                        });
                        break;
                }
            }
        }


        private void OnTriggerExit2D(Collider2D other)
        {
            //Debug.Log($"OnTriggerExit2D this:{gameObject.name}  other:{other.transform.name}");
            var mapBoundary = other.gameObject.GetComponentInParent<MapBoundary>();
            if (mapBoundary != null && mapBoundary.Type == MapBoundary.BoundaryType.MainArea)
            {
                if (other is PolygonCollider2D)
                {
                    insideParmittedArea = false;
                }
            }
        }
    }
}