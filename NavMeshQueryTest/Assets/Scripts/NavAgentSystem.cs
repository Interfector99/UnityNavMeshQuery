using UnityEngine;
using Unity.Entities;
using UnityEngine.Experimental.AI;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Burst;

[BurstCompile]
public partial struct NavAgentSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var navMeshWorld = NavMeshWorld.GetDefaultWorld();
        var query = new NavMeshQuery(navMeshWorld, Allocator.TempJob);

        // Update agents
        foreach (var (navAgent, transform, entity)
            in SystemAPI.Query<
                RefRW<NavAgentComponent>,
                RefRW<LocalTransform>>()
                .WithEntityAccess())
        {
            DynamicBuffer<WaypointBuffer> waypointBuffer = state.EntityManager.GetBuffer<WaypointBuffer>(entity);

            if (!navAgent.ValueRO.pathCalculated)
            {
                navAgent.ValueRW.pathCalculated = true;
                CalculatePath(navAgent, transform, waypointBuffer);
            }
            else
            {
                Move(navAgent, transform, waypointBuffer, query, SystemAPI.Time.DeltaTime);
            }
        }

        query.Dispose();
    }

    [BurstCompile]
    private void Move(
    RefRW<NavAgentComponent> navAgent,
    RefRW<LocalTransform> transform,
    DynamicBuffer<WaypointBuffer> waypointBuffer,
    NavMeshQuery query,
    float deltaTime)
    {
        float3 agentPosition = transform.ValueRO.Position;
        float3 targetPosition = navAgent.ValueRO.targetPosition;
        float distanceToTarget = math.distance(agentPosition, targetPosition);
        float stoppingDistance = 0.2f;

        if (distanceToTarget <= stoppingDistance)
        {
            return;
        }

        float3 desiredVelocity;

        if (waypointBuffer.Length == 0 || navAgent.ValueRO.currentWaypoint >= waypointBuffer.Length)
        {
            float3 direction = targetPosition - agentPosition;
            desiredVelocity = math.normalize(direction) * navAgent.ValueRO.moveSpeed;
        }
        else
        {
            if (math.distance(agentPosition, waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint) < stoppingDistance)
            {
                navAgent.ValueRW.currentWaypoint += 1;
            }

            if (navAgent.ValueRO.currentWaypoint < waypointBuffer.Length)
            {
                float3 direction = waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint - agentPosition;
                desiredVelocity = math.normalize(direction) * navAgent.ValueRO.moveSpeed;
            }
            else
            {
                float3 direction = targetPosition - agentPosition;
                desiredVelocity = math.normalize(direction) * navAgent.ValueRO.moveSpeed;
            }
        }

        if (math.length(desiredVelocity) > navAgent.ValueRO.moveSpeed)
        {
            desiredVelocity = math.normalize(desiredVelocity) * navAgent.ValueRO.moveSpeed;
        }

        if (!math.all(desiredVelocity == float3.zero))
        {
            float angle = math.degrees(math.atan2(desiredVelocity.x, desiredVelocity.z));
            transform.ValueRW.Rotation = quaternion.RotateY(math.radians(angle));
        }

        float3 newPosition = agentPosition + desiredVelocity * deltaTime;

        NavMeshLocation location = query.MapLocation(newPosition, new float3(2, 2, 2), 0);
        transform.ValueRW.Position = location.position;
    }

    [BurstCompile]
    private void CalculatePath(
        RefRW<NavAgentComponent> navAgent,
        RefRW<LocalTransform> transform,
        DynamicBuffer<WaypointBuffer> waypointBuffer)
    {
        NavMeshQuery query = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.TempJob, 1000);

        float3 fromPosition = transform.ValueRO.Position;
        float3 toPosition = navAgent.ValueRO.targetPosition;
        float3 extents = new float3(1, 1, 1);

        NavMeshLocation fromLocation = query.MapLocation(fromPosition, extents, 0);
        NavMeshLocation toLocation = query.MapLocation(toPosition, extents, 0);

        PathQueryStatus status;
        PathQueryStatus returningStatus;
        int maxPathSize = 100;

        if (query.IsValid(fromLocation) && query.IsValid(toLocation))
        {
            status = query.BeginFindPath(fromLocation, toLocation);
            if (status == PathQueryStatus.InProgress)
            {
                status = query.UpdateFindPath(500, out int iterationsPerformed);
                if (status == PathQueryStatus.Success)
                {
                    status = query.EndFindPath(out int pathSize);

                    NativeArray<NavMeshLocation> result = new NativeArray<NavMeshLocation>(pathSize + 1, Allocator.Temp);
                    NativeArray<StraightPathFlags> straightPathFlag = new NativeArray<StraightPathFlags>(maxPathSize, Allocator.Temp);
                    NativeArray<float> vertexSide = new NativeArray<float>(maxPathSize, Allocator.Temp);
                    NativeArray<PolygonId> polygonIds = new NativeArray<PolygonId>(pathSize + 1, Allocator.Temp);
                    int straightPathCount = 0;

                    query.GetPathResult(polygonIds);

                    returningStatus = PathUtils.FindStraightPath
                        (
                        query,
                        fromPosition,
                        toPosition,
                        polygonIds,
                        pathSize,
                        ref result,
                        ref straightPathFlag,
                        ref vertexSide,
                        ref straightPathCount,
                        maxPathSize
                        );

                    if (returningStatus == PathQueryStatus.Success)
                    {
                        waypointBuffer.Clear();

                        foreach (NavMeshLocation location in result)
                        {
                            if (location.position != Vector3.zero)
                            {
                                waypointBuffer.Add(new WaypointBuffer 
                                { 
                                    wayPoint = location.position 
                                });
                            }
                        }

                        navAgent.ValueRW.currentWaypoint = 0;
                    }
                    straightPathFlag.Dispose();
                    polygonIds.Dispose();
                    vertexSide.Dispose();
                }
            }
        }
        query.Dispose();
    }
}
