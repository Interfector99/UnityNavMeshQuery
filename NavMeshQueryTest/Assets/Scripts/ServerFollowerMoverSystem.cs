using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
[BurstCompile]
public partial struct ServerFollowerMoverSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<NavAgentComponent>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        foreach (var (navAgent, transform, entity)
                 in SystemAPI.Query<RefRW<NavAgentComponent>, RefRW<LocalTransform>>().WithEntityAccess())
        {
            DynamicBuffer<WaypointBuffer> waypointBuffer = state.EntityManager.GetBuffer<WaypointBuffer>(entity);

            if (waypointBuffer.Length > 0 && !navAgent.ValueRO.reachedDestination)
            {
                MoveEntityAlongWaypoints(navAgent, transform, waypointBuffer, ref state);
            }
        }
    }

    [BurstCompile]
    private void MoveEntityAlongWaypoints(
        RefRW<NavAgentComponent> navAgent,
        RefRW<LocalTransform> transform,
        DynamicBuffer<WaypointBuffer> waypointBuffer,
        ref SystemState state)
    {
        if (math.distance(transform.ValueRO.Position, waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint) < 0.4f)
        {
            if (navAgent.ValueRO.currentWaypoint + 1 < waypointBuffer.Length)
            {
                navAgent.ValueRW.currentWaypoint += 1;
            }
            else
            {
                navAgent.ValueRW.reachedDestination = true;
                return;
            }
        }

        float3 direction = math.normalize(waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint - transform.ValueRO.Position);
        float3 newVelocity = direction * navAgent.ValueRO.moveSpeed * SystemAPI.Time.DeltaTime;
        transform.ValueRW.Position += newVelocity;
    }
}
