using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarteavLoader : MonoBehaviour
{
    public string Address { get; private set; }
    void Start()
    {
        if (ConnectionManager.Status == ConnectionManager.ConnectionStatus.Online))
            try
            {
                string host = "localhost";
                string port = "8080";
                Address = $"http://{host}:{port}";

                var config = new HostConfiguration {RewriteLocalhost = Config.WebHost == "*"};

                Server = new NancyHost(new UnityBootstrapper(), config, new Uri(Address));
                if (!string.IsNullOrEmpty(Config.Username))
                {
                    LoginAsync();
                }
                else
                {
                    Server.Start();
                }
            }
            catch (SocketException ex)
            {
                Debug.LogException(ex);
#if UNITY_EDITOR
                UnityEditor.EditorApplication.isPlaying = false;
#else
                    // return non-zero exit code
                    Application.Quit(1);
#endif
                return;
            }
    }

    // Update is called once per frame
    void Update()
    {
    }
}