using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;

[WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
[BurstCompile]
[UpdateAfter(typeof(GoInGameServerSystem))]
public partial struct SpawnSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<EntitiesReferences>();
        state.RequireForUpdate<NetworkId>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var ecb = new EntityCommandBuffer(Allocator.Temp);

        var networkId = SystemAPI.GetSingleton<NetworkId>().Value;

        // Get the EntitiesReferences singleton which contains the follower and target prefabs
        var entitiesReferences = SystemAPI.GetSingleton<EntitiesReferences>();
        Entity followerPrefab = entitiesReferences.followerPrefab;
        Entity targetPrefab = entitiesReferences.targetPrefab;

        // Spawn a target entity at a fixed position in the scene
        float3 targetPosition = new float3(0f, 1.0f, 30f);
        Entity targetEntity = ecb.Instantiate(targetPrefab);
        ecb.SetComponent(targetEntity, new LocalTransform
        {
            Position = targetPosition,
            Rotation = quaternion.identity,
            Scale = 1f
        });
        ecb.AddComponent<Target>(targetEntity);

        // Spawn a single follower at position (20, 0, 20)
        float3 followerPosition = new float3(20f, 0f, 20f);

        Entity followerEntity = ecb.Instantiate(followerPrefab);
        ecb.SetComponent(followerEntity, new LocalTransform
        {
            Position = followerPosition,
            Rotation = quaternion.identity,
            Scale = 1f
        });
        ecb.SetComponent(followerEntity, new NavAgentComponent
        {
            targetPosition = targetPosition,
            moveSpeed = 15f,
            pathCalculated = false,
            avoidanceRadius = 0.74f,
            reachedDestination = false
        });

        // Set the GhostOwner to associate this follower with the network connection
        ecb.SetComponent(followerEntity, new GhostOwner
        {
            NetworkId = networkId
        });

        ecb.Playback(state.EntityManager);
        ecb.Dispose();

        state.Enabled = false; // Disable after spawning entities
    }
}
