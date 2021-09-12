/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using SimpleJSON;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Simulator.Api.Commands
{
    class AddRandomAgents : IDistributedCommand
    {
        public string Name => "simulator/add_random_agents";

        public void Execute(JSONNode args)
        {
            var api = ApiManager.Instance;

            if (SimulatorManager.Instance == null)
            {
                api.SendError(this, "SimulatorManager not found! Is scene loaded?");
                return;
            }

            var agentType = (AgentType)args["type"].AsInt;
            switch (agentType)
            {
                case AgentType.Npc:
                    var npcManager = SimulatorManager.Instance.NPCManager;
                    var pooledNPCs = npcManager.SpawnNPCPool();
                    if (pooledNPCs == null)
                    {
                        api.SendError(this, "No npcs to spawn");
                        return;
                    }
                    npcManager.NPCActive = true;
                    npcManager.SetNPCOnMap(true);
                    api.SendResult(this);
                    break;
                case AgentType.Pedestrian:
                    var pedManager = SimulatorManager.Instance.PedestrianManager;
                    var pooledPeds = pedManager.SpawnPedPool();
                    if (pooledPeds == null)
                    {
                        api.SendError(this, "No pedestrians to spawn");
                        return;
                    }
                    pedManager.PedestriansActive = true;
                    pedManager.SetPedOnMap(true);
                    api.SendResult(this);
                    break;
                default:
                    api.SendError(this, $"Unsupported '{args["type"]}' type");
                    break;
            }
        }

        
    }
}
