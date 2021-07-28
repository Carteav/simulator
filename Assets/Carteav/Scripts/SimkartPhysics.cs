using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimkartPhysics : MonoBehaviour, IVehicleInputs
{
    public Rigidbody Rigidbody;

    public float SteerInput { get; private set; }
    public float AccelInput { get; private set; }
    public float BrakeInput { get; private set; }
    
    private bool testing = false;

    private float testTime = 0;

    void Start()
    {
        Rigidbody = GetComponentInChildren<Rigidbody>();
    }


    void Update()
    {
        if(Input.GetKey(KeyCode.UpArrow))
        {
            AccelInput+=0.1f;
        }

        if(Input.GetKey(KeyCode.DownArrow))
        {
            AccelInput-=0.1f;
        }

        if(Input.GetKey(KeyCode.LeftArrow))
        {
            SteerInput+=0.01f;
        }

        if(Input.GetKey(KeyCode.RightArrow))
        {
            SteerInput-=0.01f;
        }

        if (Input.GetKeyDown(KeyCode.T))
        {
            if (!testing)
            {
                testing = true;
                testTime = Time.time;
                transform.position = transform.position - new Vector3(0, 0, 100);
                Debug.Log($"Starting position: {transform.position}, starting velocity: {Rigidbody.velocity}");
                Rigidbody.velocity = 10 * transform.forward;
                Debug.Log($"Velocity set to: {Rigidbody.velocity}, time: {Time.time}");
            }
        }
    }

    void FixedUpdate()
    {
        if (testing)
        {
            Rigidbody.velocity = 10 * transform.forward;
            if (Time.time - testTime > 5 && Time.time - testTime < 5.5)
            {
                Debug.Log($"Middle position: {transform.position}, middle velocity: {Rigidbody.velocity}, time: {Time.time}");
            }

            if (Time.time - testTime >= 10)
            {
                Rigidbody.velocity = Vector3.zero;
                Debug.Log($"End position: {transform.position}, end velocity: {Rigidbody.velocity}, time: {Time.time}");
                testing = false;
            }
        }
    }
}