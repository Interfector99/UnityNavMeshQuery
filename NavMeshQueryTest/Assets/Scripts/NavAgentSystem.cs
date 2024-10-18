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

        // Collect all agents
        var agentQuery = SystemAPI.QueryBuilder()
            .WithAll<NavAgentComponent, LocalTransform>()
            .Build();

        var agentEntities = agentQuery.ToEntityArray(Allocator.TempJob);
        var agentTransforms = agentQuery.ToComponentDataArray<LocalTransform>(Allocator.TempJob);

        var agentPositions = new NativeArray<float3>(agentTransforms.Length, Allocator.TempJob);
        for (int i = 0; i < agentTransforms.Length; i++)
        {
            agentPositions[i] = agentTransforms[i].Position;
        }

        // Update agents
        foreach (var (navAgent, transform, entity)
            in SystemAPI.Query<
                RefRW<NavAgentComponent>,
                RefRW<LocalTransform>>()
                .WithEntityAccess())
        {
            DynamicBuffer<WaypointBuffer> waypointBuffer = state.EntityManager.GetBuffer<WaypointBuffer>(entity);

            if (navAgent.ValueRO.nextPathCalculateTime < SystemAPI.Time.ElapsedTime)
            {
                navAgent.ValueRW.nextPathCalculateTime += 1;
                navAgent.ValueRW.pathCalculated = false;
                CalculatePath(navAgent, transform, waypointBuffer, ref state);
            }
            else
            {
                Move(navAgent, transform, waypointBuffer, ref state, entity, query, agentEntities, agentPositions);
            }
        }

        query.Dispose();
        agentEntities.Dispose();
        agentTransforms.Dispose();
        agentPositions.Dispose();
    }

    [BurstCompile]
    private void Move(
    RefRW<NavAgentComponent> navAgent,
    RefRW<LocalTransform> transform,
    DynamicBuffer<WaypointBuffer> waypointBuffer,
    ref SystemState state,
    Entity entity,
    NavMeshQuery query,
    NativeArray<Entity> agentEntities,
    NativeArray<float3> agentPositions)
    {
        // Check if the waypointBuffer has waypoints
        if (waypointBuffer.Length == 0)
        {
            // No waypoints to follow, possibly path calculation failed
            // You can choose to attempt to recalculate the path or handle this case appropriately
            return;
        }

        // Ensure currentWaypoint index is within the bounds of waypointBuffer
        if (navAgent.ValueRO.currentWaypoint >= waypointBuffer.Length)
        {
            // Reached the end of waypoints
            return;
        }

        // Check if we have reached the current waypoint
        if (math.distance(transform.ValueRO.Position, waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint) < 0.4f)
        {
            if (navAgent.ValueRO.currentWaypoint + 1 < waypointBuffer.Length)
            {
                navAgent.ValueRW.currentWaypoint += 1;
            }
            else
            {
                // Reached the last waypoint
                return;
            }
        }

        float3 direction = waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint - transform.ValueRO.Position;

        // Calculate the desired velocity towards the next waypoint
        float3 desiredDirection = math.normalize(direction);
        float3 desiredVelocity = desiredDirection * navAgent.ValueRO.moveSpeed;

        // Calculate avoidance velocity
        float3 avoidanceVelocity = CalculateAvoidanceVelocity(navAgent, transform, agentEntities, agentPositions, entity);

        // Combine the desired velocity with the avoidance velocity
        float3 combinedVelocity = desiredVelocity + avoidanceVelocity;

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
        float3 newPosition = transform.ValueRO.Position + combinedVelocity * SystemAPI.Time.DeltaTime;

        // Project the new position onto the NavMesh
        NavMeshLocation location = query.MapLocation(newPosition, new float3(2, 2, 2), 0);
        transform.ValueRW.Position = location.position;
    }


    private float3 CalculateAvoidanceVelocity(
        RefRW<NavAgentComponent> navAgent,
        RefRW<LocalTransform> transform,
        NativeArray<Entity> agentEntities,
        NativeArray<float3> agentPositions,
        Entity entity)
    {
        float3 avoidanceVelocity = float3.zero;
        int neighborCount = 0;
        float3 agentPosition = transform.ValueRO.Position;

        // Loop through all other agents
        for (int i = 0; i < agentEntities.Length; i++)
        {
            Entity otherEntity = agentEntities[i];

            // Skip self
            if (otherEntity == entity)
                continue;

            float3 otherPosition = agentPositions[i];
            float distance = math.distance(agentPosition, otherPosition);

            if (distance < navAgent.ValueRO.avoidanceRadius && distance > 0)
            {
                // Calculate avoidance direction
                float3 directionAway = agentPosition - otherPosition;
                float3 normalizedDirectionAway = math.normalize(directionAway);

                // Weight the avoidance based on distance
                float weight = (navAgent.ValueRO.avoidanceRadius - distance) / navAgent.ValueRO.avoidanceRadius;
                avoidanceVelocity += normalizedDirectionAway * weight;
                neighborCount++;
            }
        }

        if (neighborCount > 0)
        {
            avoidanceVelocity /= neighborCount;
            avoidanceVelocity = math.normalize(avoidanceVelocity) * navAgent.ValueRO.moveSpeed;
        }

        return avoidanceVelocity;
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