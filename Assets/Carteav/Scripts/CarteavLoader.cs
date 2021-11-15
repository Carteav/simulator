using UnityEngine;

public class CarteavLoader : MonoBehaviour
{
    public string Address { get; private set; }
    void Awake()
    {
        var others = FindObjectsOfType<CarteavLoader>();
        if (others.Length > 1)
        {
            Destroy(gameObject);
            return;
        }
        DontDestroyOnLoad(gameObject);
    }

    // Update is called once per frame
    void Update()
    {
    }
}