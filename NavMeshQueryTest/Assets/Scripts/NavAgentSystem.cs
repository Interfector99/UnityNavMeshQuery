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

            float3 currentTargetPosition = state.EntityManager.GetComponentData<LocalTransform>(navAgent.ValueRO.targetEntity).Position;

            // Check if the target has moved
            if (!math.all(navAgent.ValueRO.lastTargetPosition == currentTargetPosition))
            {
                // Update last known target position
                navAgent.ValueRW.lastTargetPosition = currentTargetPosition;

                // Force immediate path recalculation
                navAgent.ValueRW.nextPathCalculateTime = 0;
                navAgent.ValueRW.pathCalculated = false;
                waypointBuffer.Clear();
            }

            if (navAgent.ValueRO.nextPathCalculateTime < SystemAPI.Time.ElapsedTime)
            {
                navAgent.ValueRW.nextPathCalculateTime = (float)SystemAPI.Time.ElapsedTime + 1; // Update to recalculate every second
                navAgent.ValueRW.pathCalculated = false;
                CalculatePath(navAgent, transform, waypointBuffer, ref state);
            }
            else
            {
                Move(navAgent, transform, waypointBuffer, ref state, entity, query);
            }
        }

        query.Dispose();
    }


    [BurstCompile]
    private void Move(
    RefRW<NavAgentComponent> navAgent,
    RefRW<LocalTransform> transform,
    DynamicBuffer<WaypointBuffer> waypointBuffer,
    ref SystemState state,
    Entity entity,
    NavMeshQuery query)
    {
        float3 agentPosition = transform.ValueRO.Position;
        float3 targetPosition = state.EntityManager.GetComponentData<LocalTransform>(navAgent.ValueRO.targetEntity).Position;

        // Define a stopping distance
        float stoppingDistance = 0.5f; // Adjust as needed

        // Calculate the distance to the target
        float distanceToTarget = math.distance(agentPosition, targetPosition);

        // If the agent is within stopping distance, do nothing
        if (distanceToTarget <= stoppingDistance)
        {
            // Agent has reached the target
            return;
        }

        float3 desiredVelocity;

        // If no waypoints or reached the end of waypoints, move directly towards the target
        if (waypointBuffer.Length == 0 || navAgent.ValueRO.currentWaypoint >= waypointBuffer.Length)
        {
            // Calculate direction towards the target
            float3 direction = targetPosition - agentPosition;
            desiredVelocity = math.normalize(direction) * navAgent.ValueRO.moveSpeed;
        }
        else
        {
            // Check if we have reached the current waypoint
            if (math.distance(agentPosition, waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint) < 0.4f)
            {
                if (navAgent.ValueRO.currentWaypoint + 1 < waypointBuffer.Length)
                {
                    navAgent.ValueRW.currentWaypoint += 1;
                }
                else
                {
                    // Reached the last waypoint, start moving directly towards the target
                    navAgent.ValueRW.currentWaypoint += 1;
                }
            }

            // Ensure currentWaypoint index is within bounds
            if (navAgent.ValueRO.currentWaypoint < waypointBuffer.Length)
            {
                float3 direction = waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint - agentPosition;
                desiredVelocity = math.normalize(direction) * navAgent.ValueRO.moveSpeed;
            }
            else
            {
                // Move directly towards the target
                float3 direction = targetPosition - agentPosition;
                desiredVelocity = math.normalize(direction) * navAgent.ValueRO.moveSpeed;
            }
        }

        // Combine the desired velocity (no avoidance)
        float3 combinedVelocity = desiredVelocity;

        // Limit the combined velocity to the agent's moveSpeed
        if (math.length(combinedVelocity) > navAgent.ValueRO.moveSpeed)
        {
            combinedVelocity = math.normalize(combinedVelocity) * navAgent.ValueRO.moveSpeed;
        }

        // Rotate the agent towards the movement direction
        if (!math.all(combinedVelocity == float3.zero))
        {
            float angle = math.degrees(math.atan2(combinedVelocity.x, combinedVelocity.z));
            transform.ValueRW.Rotation = quaternion.RotateY(math.radians(angle));
        }

        // Move the agent
        float3 newPosition = agentPosition + combinedVelocity * SystemAPI.Time.DeltaTime;

        // Project the new position onto the NavMesh
        NavMeshLocation location = query.MapLocation(newPosition, new float3(2, 2, 2), 0);
        transform.ValueRW.Position = location.position;
    }

    [BurstCompile]
    private void CalculatePath(
        RefRW<NavAgentComponent> navAgent,
        RefRW<LocalTransform> transform,
        DynamicBuffer<WaypointBuffer> waypointBuffer,
        ref SystemState state)
    {
        NavMeshQuery query = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.TempJob, 1000);

        float3 fromPosition = transform.ValueRO.Position;
        float3 toPosition = state.EntityManager.GetComponentData<LocalTransform>(navAgent.ValueRO.targetEntity).Position;
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
                status = query.UpdateFindPath(100, out int iterationsPerformed);
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
                                waypointBuffer.Add(new WaypointBuffer { wayPoint = location.position });
                            }
                        }

                        navAgent.ValueRW.currentWaypoint = 0;
                        navAgent.ValueRW.pathCalculated = true;
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