using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimkartPhysics : MonoBehaviour
{
    public float wheelBase;
    Vector3 pos;
    Quaternion Myrotation;


    public float accelerationInput;
    public float SteerInput;
    public bool reverse;
    float currentVelocitcy;
    float currentOri;
    Vector3 angles;

 //   public Transform originTrans;

    void Start()
    {
        pos = transform.position;
        Myrotation = transform.rotation;
   ////     originTrans.position = transform.position;
        angles = Myrotation.eulerAngles;
    }

    void Update()
    {
        if(Input.GetKey("taA                                                                                                                                                                                                                                                                                                                                                                                                                                                                "))
        {
            accelerationInput+=0.1f;
        }

        if(Input.GetKey("f"))
        {
            accelerationInput-=0.1f;
        }

        if(Input.GetKey("d"))
        {
            SteerInput+=0.1f;
        }

        if(Input.GetKey("a"))
        {
            SteerInput-=0.1f;
        }

    }

    void FixedUpdate()
    {
        ApplySteer();
        ApplyAcceleration();
    }

    void ApplySteer()
    {
        angles.y += currentVelocitcy * Mathf.Tan(SteerInput) / wheelBase * Time.deltaTime;

        pos.x += currentVelocitcy * Mathf.Cos(angles.y) * Time.deltaTime;
        pos.z += currentVelocitcy * Mathf.Sin(angles.y) * Time.deltaTime;
        transform.position = pos;


        transform.rotation = Quaternion.AngleAxis((-180*angles.y)/3.14159f, Vector3.up); 
    }

    void ApplyAcceleration()
    {
           currentVelocitcy += accelerationInput* Time.deltaTime;
    }

}