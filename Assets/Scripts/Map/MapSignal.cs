/**
 * Copyright (c) 2019-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;
using Simulator.Utilities;
using Simulator.Controllable;
using Simulator.Network.Core.Messaging;

namespace Simulator.Map
{
    public class MapSignal : MapData, IControllable, IMapType
    {
        public bool Spawned { get; set; } = false;
        public uint SeqId;
        public Vector3 boundOffsets = new Vector3();
        public Vector3 boundScale = new Vector3();
        public List<SignalData> signalData = new List<SignalData>();
        public MapLine stopLine;
        public SignalLight CurrentSignalLight;
        public SignalType signalType = SignalType.MIX_3_VERTICAL;
        private Coroutine SignalCoroutine;
        private MessagesManager messagesManager;
        
        [SerializeField] private string _UID;
        public string UID
        {
            get { return _UID; }
            set
            {
#if UNITY_EDITOR
                Undo.RecordObject(this, "Changed signal UID");
#endif
                _UID = value;
            }
        }

        [SerializeField] private string _id;
        public string id
        {
            get { return _id; }
            set
            {
#if UNITY_EDITOR
                Undo.RecordObject(this, "Changed signal ID");
#endif
                _id = value;
            }
        }

        public string Key => UID;

        public string GUID => UID;
        public string ControlType { get; set; } = "signal";
        public string CurrentState { get; set; }
        public string[] ValidStates { get; set; } = new string[] { "green", "yellow", "red", "black" };
        public string[] ValidActions { get; set; } = new string[] { "trigger", "wait", "loop" };
        public List<ControlAction> DefaultControlPolicy { get; set; } = new List<ControlAction>();
        public List<ControlAction> CurrentControlPolicy { get; set; }

        static int MaxId = -1; // Maximum id of existing signals

 #if UNITY_EDITOR
        private void Reset()
        {
            id = "signal_" + (++MaxId);
            if (string.IsNullOrEmpty(UID))
            {
                UID = Guid.NewGuid().ToString();
            }
        }

        [InitializeOnLoadMethod]
        static void Initialize()
        {
            UnityEditor.SceneManagement.EditorSceneManager.sceneOpened += OnEditorSceneManagerSceneOpened;
        }

        static void OnEditorSceneManagerSceneOpened(UnityEngine.SceneManagement.Scene scene, UnityEditor.SceneManagement.OpenSceneMode mode)
        {
            var mapHolder = UnityEngine.Object.FindObjectOfType<MapHolder>();
            if (mapHolder == null)
                return;

            var existingSignals = new List<MapSignal>(mapHolder.transform.GetComponentsInChildren<MapSignal>());
            int curMaxId = 0;
            for (int i = 0; i < existingSignals.Count; i++)
            {
                var signal = existingSignals[i];
                int curId = GetNumber(signal.id);
                if (curId > curMaxId)
                {
                    curMaxId = curId;
                }
                if (string.IsNullOrEmpty(signal.UID)) // if there is no UID set, set a random one
                {
                    signal.UID = Guid.NewGuid().ToString();
                }
            }
            MaxId = curMaxId;
        }
 #endif
        static int GetNumber(string signalId)
        {
            var splitted = signalId.Split('_');
            if (splitted.Length < 2) return 0;

            return int.TryParse(splitted[1], out int num) ? num : 0;
        }

        public void Control(List<ControlAction> controlActions)
        {
            var fixedUpdateManager = SimulatorManager.Instance.FixedUpdateManager;

            if (SignalCoroutine != null)
            {
                fixedUpdateManager.StopCoroutine(SignalCoroutine);
                SignalCoroutine = null;
            }

            SignalCoroutine = fixedUpdateManager.StartCoroutine(SignalLoop(controlActions));
        }

        private void OnDestroy()
        {
            Resources.UnloadUnusedAssets();
        }

        public void SetSignalMeshData()
        {
            var signalLights = new List<SignalLight>();
            signalLights.AddRange(FindObjectsOfType<SignalLight>());
            foreach (var light in signalLights)
            {
                if (Vector3.Distance(transform.position, light.transform.position) < 0.1f)
                {
                    CurrentSignalLight = light;
                    break;
                }
            }
        }

        private Color GetTypeColor(SignalData data)
        {
            Color currentColor = Color.black;
            switch (data.signalColor)
            {
                case SignalColorType.Red:
                    currentColor = Color.red;
                    break;
                case SignalColorType.Yellow:
                    currentColor = Color.yellow;
                    break;
                case SignalColorType.Green:
                    currentColor = Color.green;
                    break;
                default:
                    break;
            }
            return currentColor;
        }

        public void SetSignalState(string state)
        {
            if (!ValidStates.Contains(state))
            {
                Debug.LogWarning($"'{state}' is an invalid state for '{ControlType}'");
                return;
            }

            CurrentState = state;
            CurrentSignalLight.SetSignalLightState(state);
            switch (CurrentState)
            {
                case "red":
                    stopLine.currentState = SignalLightStateType.Red;
                    break;
                case "green":
                    stopLine.currentState = SignalLightStateType.Green;
                    break;
                case "yellow":
                    stopLine.currentState = SignalLightStateType.Yellow;
                    break;
                case "black":
                    stopLine.currentState = SignalLightStateType.Black;
                    break;
            }
        }

        private IEnumerator SignalLoop(List<ControlAction> controlActions)
        {
            var fixedUpdateManager = SimulatorManager.Instance.FixedUpdateManager;

            for (int i = 0; i < controlActions.Count; i++)
            {
                var action = controlActions[i].Action;
                var value = controlActions[i].Value;

                switch (action)
                {
                    case "state":
                        SetSignalState(value);
                        SimulatorManager.Instance.ControllableManager.DistributeCommand(this, controlActions[i]);
                        break;
                    case "trigger":
                        if (!float.TryParse(value, out float threshold) || threshold < 0f)
                        {
                            threshold = 0f;
                        }
                        yield return fixedUpdateManager.WaitUntilFixed(() => IsAgentAround(threshold));
                        break;
                    case "wait":
                        if (!float.TryParse(value, out float seconds) || seconds < 0f)
                        {
                            seconds = 0f;
                        }
                        yield return fixedUpdateManager.WaitForFixedSeconds(seconds);
                        break;
                    case "loop":
                        i = -1;
                        break;
                    default:
                        Debug.LogWarning($"'{action}' is an invalid action for '{ControlType}'");
                        break;
                }
            }
        }

        private bool IsAgentAround(float threshold)
        {
            var agent = SimulatorManager.Instance.AgentManager.CurrentActiveAgent;
            if (agent == null)
            {
                return false;
            }

            var agentPos = agent.transform.position;
            var signalPos = new Vector3(transform.position.x, agentPos.y, transform.position.z);  // Check distance in xz plane
            var distance = Vector3.Distance(agentPos, signalPos);

            return distance <= threshold;
        }

        public System.ValueTuple<Vector3, Vector3, Vector3, Vector3> Get2DBounds()
        {
            var matrix = transform.localToWorldMatrix * Matrix4x4.TRS(boundOffsets, Quaternion.identity, Vector3.Scale(Vector3.one, boundScale));

            float min = boundScale[0];
            int index = 0;
            for (int i = 0; i < 3; i++)
            {
                if (boundScale[i] < min)
                {
                    min = boundScale[i];
                    index = i;
                }
            }

            if (index == 0)
            {
                return ValueTuple.Create(
                    matrix.MultiplyPoint(new Vector3(0, 0.5f, 0.5f)),
                    matrix.MultiplyPoint(new Vector3(0, -0.5f, 0.5f)),
                    matrix.MultiplyPoint(new Vector3(0, -0.5f, -0.5f)),
                    matrix.MultiplyPoint(new Vector3(0, 0.5f, -0.5f))
                    );
            }
            else if (index == 1)
            {
                return ValueTuple.Create(
                    matrix.MultiplyPoint(new Vector3(0.5f, 0, 0.5f)),
                    matrix.MultiplyPoint(new Vector3(-0.5f, 0, 0.5f)),
                    matrix.MultiplyPoint(new Vector3(-0.5f, 0, -0.5f)),
                    matrix.MultiplyPoint(new Vector3(0.5f, 0, -0.5f))
                    );
            }
            else
            {
                return ValueTuple.Create(
                    matrix.MultiplyPoint(new Vector3(0.5f, 0.5f, 0)),
                    matrix.MultiplyPoint(new Vector3(-0.5f, 0.5f, 0)),
                    matrix.MultiplyPoint(new Vector3(-0.5f, -0.5f, 0)),
                    matrix.MultiplyPoint(new Vector3(0.5f, -0.5f, 0))
                    );
            }
        }

        public override void Draw()
        {
            if (signalData == null || signalData.Count < 1)
                return;

            var lightLocalPositions = signalData.Select(x => x.localPosition).ToList();
            var lightCount = lightLocalPositions.Count;

            // lights
            if (MapAnnotationTool.SHOW_HELP)
            {
#if UNITY_EDITOR
                UnityEditor.Handles.Label(transform.position, "    SIGNAL");
#endif
            }

            for (int i = 0; i < lightCount; i++)
            {
                var start = transform.TransformPoint(lightLocalPositions[i]);
                var end = start + transform.forward * 2f * (1 / MapAnnotationTool.EXPORT_SCALE_FACTOR); // TODO why is this 1/export scale?

                var signalColor = GetTypeColor(signalData[i]) + selectedColor;

                AnnotationGizmos.DrawWaypoint(start, MapAnnotationTool.PROXIMITY * 0.15f, signalColor);
                Gizmos.color = signalColor;
                Gizmos.DrawLine(start, end);
                AnnotationGizmos.DrawArrowHead(start, end, signalColor, arrowHeadScale: MapAnnotationTool.ARROWSIZE, arrowPositionRatio: 1f);
            }

            // stopline
            if (stopLine != null)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(transform.position, stopLine.transform.position);
                AnnotationGizmos.DrawArrowHead(transform.position, stopLine.transform.position, Color.magenta, arrowHeadScale: MapAnnotationTool.ARROWSIZE, arrowPositionRatio: 1f);
                if (MapAnnotationTool.SHOW_HELP)
                {
#if UNITY_EDITOR
                    UnityEditor.Handles.Label(stopLine.transform.position, "    STOPLINE");
#endif
                }
            }

            // bounds
            Gizmos.matrix = transform.localToWorldMatrix * Matrix4x4.TRS(Vector3.zero, Quaternion.identity, Vector3.Scale(Vector3.one, boundScale));
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
            if (MapAnnotationTool.SHOW_HELP)
            {
#if UNITY_EDITOR
                UnityEditor.Handles.Label(transform.position + Vector3.up, "    SIGNAL BOUNDS");
#endif
            }
        }
    }
}
