using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using Unity.Mathematics;
using Unity.Transforms;

[WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
[BurstCompile]
public partial struct ServerWaypointReceiverSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<WaypointRpcCommand>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var ecb = new EntityCommandBuffer(Allocator.Temp);

        foreach (var (rpcCommand, rpcRequest, entity)
                 in SystemAPI.Query<RefRO<WaypointRpcCommand>, RefRO<ReceiveRpcCommandRequest>>().WithEntityAccess())
        {
            int ghostId = rpcCommand.ValueRO.GhostId;
            float3 waypoint = rpcCommand.ValueRO.Waypoint;

            // Find the follower entity with the matching ghostId
            foreach (var (follower, followerEntity)
                     in SystemAPI.Query<
                         RefRW<NavAgentComponent>>()
                         .WithEntityAccess())
            {
                var ghostInstance = SystemAPI.GetComponent<GhostInstance>(followerEntity);
                if (ghostInstance.ghostId == ghostId)
                {
                    var waypointBuffer = SystemAPI.GetBuffer<WaypointBuffer>(followerEntity);
                    waypointBuffer.Add(new WaypointBuffer { wayPoint = waypoint });
                    break;
                }
            }

            ecb.DestroyEntity(entity); // Clean up RPC entity after processing
        }

        ecb.Playback(state.EntityManager);
        ecb.Dispose();
    }
}
