/**
 * Copyright (c) 2019-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using UnityEngine;
using Simulator.Api;
using Simulator.Utilities;
using SimpleJSON;

class DrunkDriverControl : ICommand
{
    public string Name => "agent/drunk/config";

    public void Execute(JSONNode args)
    {
        var uid = args["uid"].Value;
        var api = ApiManager.Instance;


        if (!api.Agents.TryGetValue(uid, out GameObject npc))
        {
            api.SendError(this, $"Agent '{uid}' not found");
            return;
        }

        var behaviour = npc.GetComponent<NPCDrunkDriverBehaviour>();
        if (behaviour == null)
        {
            api.SendError(this, $"Agent '{uid}' is not a drunk driving NPC agent");
            return;
        }

        if (args.HasKey("correctionMinTime"))
        {
            behaviour.steerCorrectionMinTime = args["correctionMinTime"].AsFloat;
        }
        if (args.HasKey("correctionMaxTime"))
        {
            behaviour.steerCorrectionMaxTime = args["correctionMaxTime"].AsFloat;
        }
        if (args.HasKey("steerDriftMin"))
        {
            behaviour.steerDriftMin = args["steerDriftMin"].AsFloat;
        }
        if (args.HasKey("steerDriftMax"))
        {
            behaviour.steerDriftMax = args["steerDriftMax"].AsFloat;
        }
        api.SendResult(this);
    }
}

public class NPCDrunkDriverBehaviour : NPCLaneFollowBehaviour
{
    public float steerCorrectionMinTime = 0.0f;
    public float steerCorrectionMaxTime = 0.4f;

    public float steerDriftMin = 0.00f;
    public float steerDriftMax = 0.09f;
    
    protected float currentSteerDrift = 0.0f;
    protected float nextSteerCorrection = 0;

    protected override void SetTargetTurn()
    {
        if (nextSteerCorrection < Time.fixedTime)
        {
            float steerCorrectionIn = RandomGenerator.NextFloat(steerCorrectionMinTime, steerCorrectionMaxTime);
            nextSteerCorrection = Time.fixedTime + steerCorrectionIn;

            currentSteerDrift = RandomGenerator.NextFloat(steerDriftMin, steerDriftMax);
            currentSteerDrift = currentSteerDrift * Mathf.Abs(RandomGenerator.NextFloat(-1.0f, 1.0f));
            base.SetTargetTurn();
        }
        else 
        {
            currentTurn += currentSteerDrift * currentSpeed;
        }
    }
}