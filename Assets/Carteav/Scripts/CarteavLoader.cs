using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarteavLoader : MonoBehaviour
{
    public string Address { get; private set; }
    void Awake()
    {
        DontDestroyOnLoad(gameObject);
    }

    // Update is called once per frame
    void Update()
    {
    }
}