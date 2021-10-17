/**
 * Copyright (c) 2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Simulator;
using Simulator.Map;
using Simulator.Network.Core.Components;
using Random = UnityEngine.Random;

public class ParkingNPCManager : NPCManager, ICustomManager
{
    private const float fillRateDiff = 0.05f;
    private const float MinimumAmountOfTimeCarMustBeParkedDefault = 100000;
    public static bool NPCParkingBehaviourEnabled = true;
    public static float MinimumAmountOfTimeCarMustBeParked = MinimumAmountOfTimeCarMustBeParkedDefault;

    private float maxParkingFillRate = 1.4f;

    private float spawnPause = 5;
    private float despawnPause = 5;

    //per minute
    public int SpawnRate
    {
        set => spawnPause = value / 60f;
    }

    public int DepawnRate
    {
        set => despawnPause = value / 60f;
    }

    private SpawnsManager parkingSpawnsManager;

    private int extraNPCPoolSize;
    protected override int NPCPoolSize => base.NPCPoolSize + extraNPCPoolSize;
    protected override Type DefaultBehaviour => typeof(NPCParkingBehaviour);

    private List<NPCParkingBehaviour> waitingBehaviours = new List<NPCParkingBehaviour>();

    private void Awake()
    {
        parkingSpawnsManager = gameObject.AddComponent<ParkingSpawnsManager>();
        extraNPCPoolSize = (int)(maxParkingFillRate * ParkingManager.instance.AllSpaces);
    }

    public override void SetNPCOnMap(bool isInitialSpawn = false)
    {
        if (!InitialSpawn)
        {
            isInitialSpawn = true;
        }
        base.SetNPCOnMap(isInitialSpawn);

        if (isInitialSpawn)
        {
            TrySpawn(CurrentPooledNPCs.Count);
        }
    }


    private void TrySpawn(int count, int startIndex = 0, MapParkingSpace overrideSpace = null)
    {
        var sentinel = CurrentPooledNPCs.Count;
        var i = startIndex - 1;
        while (count > 0)
        {
            i++;
            if (sentinel-- <= 0) return;
            if (i >= CurrentPooledNPCs.Count)
            {
                i = 0;
            }
            var currentNPC = CurrentPooledNPCs[i];
            if (currentNPC.gameObject.activeInHierarchy)
                continue;
            var spawnPoint =
                parkingSpawnsManager.GetValidSpawnPoint(currentNPC.Bounds, false); // we do not spwan on road
            if (spawnPoint == null)
                return;
            var parking = (spawnPoint.lane as MapParkingSpace);

            if (overrideSpace != null)
            {
                spawnPoint = new SpawnsManager.SpawnPoint()
                {
                    position = overrideSpace.Center,
                    spawnIndex = 0,
                    lookAtPoint = overrideSpace.MiddleExit,
                    lane = overrideSpace
                };
                parking = overrideSpace;
            }

            if (!ParkingManager.instance.TryTake(parking))
                continue;
            if (ParkingManager.instance.Fillrate >= maxParkingFillRate)
            {
                return;
            }
            if (currentNPC.Bounds.size.z > parking.Length)
            {
                continue;
            }

            currentNPC.transform.position = spawnPoint.position;
            currentNPC.transform.LookAt(spawnPoint.lookAtPoint);
            currentNPC.GTID = ++SimulatorManager.Instance.GTIDs;
            currentNPC.gameObject.SetActive(true);
            var behaviour = (NPCParkingBehaviour)currentNPC.ActiveBehaviour;
            behaviour.SwitchedToParked(false,
                spawnPoint.lane as MapParkingSpace);

            StartCoroutine(TurnOffRigidBody(currentNPC));

            //Force snapshots resend after changing the transform position
            if (Loader.Instance.Network.IsMaster)
            {
                var rb = CurrentPooledNPCs[i].GetComponent<DistributedRigidbody>();
                if (rb != null)
                {
                    rb.BroadcastSnapshot(true);
                }
            }
            count--;
        }
    }

    private IEnumerator DespawnWorker()
    {
        while (true)
        {
            yield return new WaitForSeconds(despawnPause);
            if (ParkingManager.instance.Fillrate >= maxParkingFillRate - fillRateDiff)
            {
                int index = Random.Range(0, CurrentPooledNPCs.Count);

                var currentNPC = CurrentPooledNPCs[index];
                if (!currentNPC.gameObject.activeInHierarchy)
                    continue;
                var behaviour = currentNPC.ActiveBehaviour as NPCParkingBehaviour;
                if (behaviour.CurrentState == NPCParkingBehaviour.State.IsParked)
                {
                    behaviour.Despawn();
                }
            }

        }
    }

    private IEnumerator SpawnWorker()
    {
        while (true)
        {
            yield return new WaitForSeconds(spawnPause);
            if (ParkingManager.instance.Fillrate < maxParkingFillRate + fillRateDiff)
            {
                int index = Random.Range(0, CurrentPooledNPCs.Count);
                TrySpawn(1, index);
            }
        }
    }

    private IEnumerator LeavingParkingWorker()
    {
        int index = 0;
        while (true)
        {
            yield return new WaitForSeconds(1f);
            if (ParkingManager.instance.Fillrate >= maxParkingFillRate - fillRateDiff)
            {
                for (; index < CurrentPooledNPCs.Count; index++)
                {
                    var p = CurrentPooledNPCs[index];
                    var park = p.ActiveBehaviour as NPCParkingBehaviour;
                    if (park.CurrentState == NPCParkingBehaviour.State.IsParked)
                    {
                        if (park.TryInitLeaving())
                        {
                            break;
                        }
                    }
                }
                if (index == CurrentPooledNPCs.Count) index = 0;
            }

        }
    }

    public override void DespawnNPC(NPCController npc)
    {
        var parkingBehaviour = (npc.ActiveBehaviour as NPCParkingBehaviour);
        npc.gameObject.SetActive(false);
        npc.transform.position = transform.position;
        npc.transform.rotation = Quaternion.identity;

        npc.StopNPCCoroutines();
        npc.enabled = false;

        if (NPCActive && parkingBehaviour.CurrentState != NPCParkingBehaviour.State.IsParked)
        {
            ActiveNPCCount--;
        }

        foreach (var callback in DespawnCallbacks)
        {
            callback(npc);
        }
    }


    IEnumerator TurnOffRigidBody(NPCController controller)
    {
        yield return new WaitForSeconds(0.5f);
        (controller.ActiveBehaviour as NPCParkingBehaviour).ChangePhysic(false);
    }

    public void ChangeActiveCountBy(int diff)
    {
        ActiveNPCCount += diff;
    }

    #region Messages
    void SetNPCParkingBehaviourEnabledMessage(bool value)
    {
        NPCParkingBehaviourEnabled = value;
    }
    void SetMinimumAmountOfTimeCarMustBeParkedMessage(float value)
    {
        MinimumAmountOfTimeCarMustBeParked = value;
    }
    void ResetMinimumAmountOfTimeCarMustBeParkedMessage()
    {
        MinimumAmountOfTimeCarMustBeParked = MinimumAmountOfTimeCarMustBeParkedDefault;
    }

    void SpaceChangedMessage(MapParkingSpace space)
    {
        var npc = GetNpcInSpace(space);
        if (npc != null)
        {
            npc.transform.position = space.Center;
            npc.transform.LookAt(space.MiddleExit);
        }
    }

    void ForceSpaceFreeMessage(MapParkingSpace space)
    {
        var npc = GetNpcInSpace(space);
        if (npc != null)
        {
            npc.ForceDespawn();
        }
        else
        {
            Debug.LogError("could not get NPC for despawn");
        }
    }

    void LeaveSpaceFreeMessage(MapParkingSpace space)
    {
        var npc = GetNpcInSpace(space);
        if (npc != null) npc.TryInitLeaving(true);
        else
        {
            Debug.LogError("could not get NPC");
        }
    }

    void SetFillRateMessage(float rate)
    {
        maxParkingFillRate = rate;
    }
    void EnableWorkersMessage(bool areEnabled)
    {
        if (areEnabled)
        {
            StartCoroutine(SpawnWorker());
            StartCoroutine(DespawnWorker());
            StartCoroutine(LeavingParkingWorker());
        }
        else
        {
            Debug.Log("StopAllCoroutines");
            StopAllCoroutines();
        }
    }
    public void FillSpaceMessage(MapParkingSpace space)
    {
        TrySpawn(1, 0, space);
    }
    public void FillAllMessage()
    {
        ParkingManager.instance.Reset();
        TrySpawn(CurrentPooledNPCs.Count);
    }


    public void ForceSomeNPCToParkInSpaceAndWaitMessage(MapParkingSpace space)
    {
        ParkingManager.instance.TryTake(space);
        var npc = ForceSomeNPCToParkInSpaceMessage(space);
        npc.Wait = true;
        waitingBehaviours.Add(npc);
    }

    public void ReleaseWaitingNPCsMessage()
    {
        foreach (var npc in waitingBehaviours)
        {
            npc.Wait = false;
        }
        waitingBehaviours.Clear();
    }
    public NPCParkingBehaviour ForceSomeNPCToParkInSpaceMessage(MapParkingSpace space)
    {
        lastTimeInit = Time.time;
        lastSpace = space;
        var lane = SimulatorManager.Instance.MapManager.GetClosestLane(space.transform.position);
        var allNPCs = SimulatorManager.Instance.NPCManager.CurrentPooledNPCs;
        var startOfLane = lane.mapWorldPositions[0];
        var minDist = Single.MaxValue;
        NPCParkingBehaviour bestNpc = null;
        var distFromSpace = (startOfLane - space.transform.position).magnitude;
        foreach (var npcController in allNPCs)
        {
            var parking = npcController.ActiveBehaviour as NPCParkingBehaviour;
            if (parking != null)
            {
                if (npcController.isActiveAndEnabled)
                {
                    if (parking.CurrentState == NPCParkingBehaviour.State.IsParking)
                    {
                        if (parking.CurrentSpace == space)
                        {
                            Debug.Log("already parking");
                            return parking;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else if (parking.CurrentState == NPCParkingBehaviour.State.IsLeaving)
                    {
                        continue;
                    }
                    var distFromLaneStart = (startOfLane - parking.transform.position).magnitude;
                    var distToParkingSpace = (space.transform.position - parking.transform.position).magnitude;
                    if (parking.currentMapLane == lane && distFromLaneStart < distFromSpace &&
                        distToParkingSpace > 10 ||
                        lane.prevConnectedLanes.Contains(parking.currentMapLane) && distFromLaneStart < 10)

                    {
                        if (distFromLaneStart < minDist)
                        {
                            bestNpc = parking;
                            minDist = distFromLaneStart;
                        }
                    }
                }
            }
        }

        if (bestNpc == null)
        {
            var npc = SpawnNpcAbleToPark(lane);
            if (npc == null)
            {
                Debug.Log("could not find free npc");
                return null; //could try again after some time
            }
            bestNpc = npc.ActiveBehaviour as NPCParkingBehaviour;

        }
        bool send = false;
        foreach (Transform child in SimulatorManager.Instance.AgentManager.CurrentActiveAgent.transform)
        {
            if (child.name.StartsWith("NPCControlSensor"))
            {
                child.gameObject.SendMessage("DoNotTrackNPCMessage", bestNpc.controller);
                send = true;
                break;
            }
        }
        if (!send)
        {
            throw new Exception("No NPCControlSensor attached");
        }
        bestNpc.InitParking(space);
        selectedNPC = bestNpc;
        return bestNpc;
    }
    #endregion

    NPCParkingBehaviour GetNpcInSpace(MapParkingSpace space)
    {
        var allNPCs = SimulatorManager.Instance.NPCManager.CurrentPooledNPCs;
        foreach (var npcController in allNPCs)
        {
            var parking = npcController.ActiveBehaviour as NPCParkingBehaviour;
            if (parking != null && parking.CurrentSpace == space && parking.CurrentState == NPCParkingBehaviour.State.IsParked)
            {
                return parking;
            }
        }
        return null;
    }
    NPCController SpawnNpcAbleToPark(MapTrafficLane lane)
    {
        if (Physics.CheckSphere(lane.mapWorldPositions[0], 3, SpawnsManager.NPCSpawnCheckBitmask)
        ) // TODO check box with npc bounds
            return null;
        for (int i = 0; i < CurrentPooledNPCs.Count; i++)
        {
            if (CurrentPooledNPCs[i].gameObject.activeInHierarchy)
                continue;
            var parking = CurrentPooledNPCs[i].ActiveBehaviour as NPCParkingBehaviour;
            if (parking == null || !parking.AbleToPark)
                continue;
            CurrentPooledNPCs[i].transform.position = lane.mapWorldPositions[0];
            CurrentPooledNPCs[i].transform.LookAt(lane.mapWorldPositions[1]);
            CurrentPooledNPCs[i].InitLaneData(lane);
            CurrentPooledNPCs[i].GTID = ++SimulatorManager.Instance.GTIDs;
            CurrentPooledNPCs[i].gameObject.SetActive(true);
            CurrentPooledNPCs[i].enabled = true;
            ActiveNPCCount++;

            //Force snapshots resend after changing the transform position
            if (Loader.Instance.Network.IsMaster)
            {
                var rb = CurrentPooledNPCs[i].GetComponent<DistributedRigidbody>();
                if (rb != null)
                {
                    rb.BroadcastSnapshot(true);
                }
            }
            return CurrentPooledNPCs[i];
        }
        Debug.LogError("Could not find npc");
        return null;
    }



    private float lastTimeInit;
    private NPCParkingBehaviour selectedNPC;
    private MapParkingSpace lastSpace;

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        if (Time.time - lastTimeInit < 0.5f) Gizmos.DrawCube(lastSpace.transform.position, Vector3.one * 2);
        if (selectedNPC != null && selectedNPC.CurrentState == NPCParkingBehaviour.State.IsParking)
        {
            Gizmos.DrawSphere(selectedNPC.transform.position, 1);
            Gizmos.DrawLine(selectedNPC.transform.position, lastSpace.transform.position);
        }
    }
}
