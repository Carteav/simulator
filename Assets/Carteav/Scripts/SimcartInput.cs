using UnityEngine;

namespace Carteav
{


    public class SimcartInput : MonoBehaviour, IVehicleInputs
    {
        //public Rigidbody Rigidbody;

        public float SteerInput { get;  set; }
        public float AccelInput { get;  set; }
        public float BrakeInput { get;  set; }

        private bool _testing = false;

        private float _testTime = 0;

        

        void Start()
        {
            //Rigidbody = GetComponentInChildren<Rigidbody>();
        }


        void Update()
        {
            if (Input.GetKey(KeyCode.UpArrow))
            {
                AccelInput += 0.1f;
            }

            if (Input.GetKey(KeyCode.DownArrow))
            {
                AccelInput -= 0.1f;
            }

            if (Input.GetKey(KeyCode.LeftArrow))
            {
                SteerInput += 0.01f;
            }

            if (Input.GetKey(KeyCode.RightArrow))
            {
                SteerInput -= 0.01f;
            }

            
        }

       

        
    }

    
    
}