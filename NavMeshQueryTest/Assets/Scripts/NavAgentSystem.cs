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
    private NavMeshQuery navMeshQuery;
    private bool navMeshQueryInitialized;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        navMeshQueryInitialized = false;
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Initialize NavMeshQuery if it hasn't been initialized
        if (!navMeshQueryInitialized)
        {
            var navMeshWorld = NavMeshWorld.GetDefaultWorld();
            navMeshQuery = new NavMeshQuery(navMeshWorld, Allocator.Persistent); // Persistent allocator for long-lived use
            navMeshQueryInitialized = true;
        }

        // Collect all agents and their positions
        var agentQuery = SystemAPI.QueryBuilder()
            .WithAll<NavAgentComponent, LocalTransform>()
            .Build();

        var agentEntities = agentQuery.ToEntityArray(Allocator.TempJob);
        var agentTransforms = agentQuery.ToComponentDataArray<LocalTransform>(Allocator.TempJob);

        // Assign unique grid positions to each agent
        AssignGridPositions(agentEntities, ref state);

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

            if (!navAgent.ValueRO.pathCalculated)
            {
                navAgent.ValueRW.pathCalculated = true;
                CalculatePath(navAgent, transform, entity, waypointBuffer);
            }
            else
            {
                Move(navAgent, transform, waypointBuffer, ref state, entity, navMeshQuery, agentEntities, agentPositions);
            }
        }

        // Dispose temporary allocations
        agentEntities.Dispose();
        agentTransforms.Dispose();
        agentPositions.Dispose();
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state)
    {
        // Dispose of NavMeshQuery on system destroy
        if (navMeshQueryInitialized)
        {
            navMeshQuery.Dispose();
        }
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
        float3 targetPosition = CalculateGridTargetPosition(navAgent.ValueRO);

        // Check if the destination has been reached
        if (navAgent.ValueRO.reachedDestination)
        {
            // Only apply avoidance logic if reachedDestination is true
            float3 avoidanceVelocityOnly = CalculateAvoidanceVelocity(navAgent, transform, agentEntities, agentPositions, entity);

            // If avoidanceVelocityOnly is non-zero, apply it to let the unit avoid
            if (!math.all(avoidanceVelocityOnly == float3.zero))
            {
                float3 avoidanceNewPosition = transform.ValueRO.Position + avoidanceVelocityOnly * SystemAPI.Time.DeltaTime;
                NavMeshLocation avoidanceLocation = query.MapLocation(avoidanceNewPosition, new float3(2, 2, 2), 0);
                transform.ValueRW.Position = avoidanceLocation.position;
            }

            // Check if the unit deviated from the target due to avoidance
            if (math.distance(transform.ValueRO.Position, targetPosition) > 0.4f)
            {
                // Move back toward the assigned grid position
                float3 returnDirection = math.normalize(targetPosition - transform.ValueRO.Position);
                float3 returnVelocity = returnDirection * navAgent.ValueRO.moveSpeed;

                // Limit return velocity if necessary
                if (math.length(returnVelocity) > navAgent.ValueRO.moveSpeed)
                {
                    returnVelocity = math.normalize(returnVelocity) * navAgent.ValueRO.moveSpeed;
                }

                // Move back to grid position
                float3 finalReturnPosition = transform.ValueRO.Position + returnVelocity * SystemAPI.Time.DeltaTime;
                NavMeshLocation finalReturnLocation = query.MapLocation(finalReturnPosition, new float3(2, 2, 2), 0);
                transform.ValueRW.Position = finalReturnLocation.position;
            }

            return; // Skip the rest of Move if destination is reached
        }

        // Ensure there are waypoints before continuing
        if (waypointBuffer.Length == 0) return;

        // Check if we have reached the current waypoint
        if (math.distance(transform.ValueRO.Position, waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint) < 0.4f)
        {
            if (navAgent.ValueRO.currentWaypoint + 1 < waypointBuffer.Length)
            {
                navAgent.ValueRW.currentWaypoint += 1;
            }
            else
            {
                // All waypoints are reached, set reachedDestination to true
                navAgent.ValueRW.reachedDestination = true;
                return;
            }
        }

        // Calculate movement toward the next waypoint
        float3 direction = waypointBuffer[navAgent.ValueRO.currentWaypoint].wayPoint - transform.ValueRO.Position;
        float3 desiredDirection = math.normalize(direction);
        float3 desiredVelocity = desiredDirection * navAgent.ValueRO.moveSpeed;

        // Calculate avoidance velocity and combine with desired velocity
        float3 combinedAvoidanceVelocity = CalculateAvoidanceVelocity(navAgent, transform, agentEntities, agentPositions, entity);
        float3 combinedVelocity = desiredVelocity + combinedAvoidanceVelocity;

        // Limit combined velocity to moveSpeed
        if (math.length(combinedVelocity) > navAgent.ValueRO.moveSpeed)
        {
            combinedVelocity = math.normalize(combinedVelocity) * navAgent.ValueRO.moveSpeed;
        }

        // Rotate to face movement direction
        if (!math.all(combinedVelocity == float3.zero))
        {
            float angle = math.degrees(math.atan2(combinedVelocity.x, combinedVelocity.z));
            transform.ValueRW.Rotation = quaternion.RotateY(math.radians(angle));
        }

        // Move the agent
        float3 newPositionFinal = transform.ValueRO.Position + combinedVelocity * SystemAPI.Time.DeltaTime;
        NavMeshLocation finalLocation = query.MapLocation(newPositionFinal, new float3(2, 2, 2), 0);
        transform.ValueRW.Position = finalLocation.position;
    }

    float3 CalculateGridTargetPosition(NavAgentComponent navAgent)
    {
        // Offset the main target position by the grid position
        float spacing = 1.1f; // Adjust spacing as needed
        float3 gridOffset = new float3(navAgent.gridPosition.x * spacing, 0, navAgent.gridPosition.y * spacing);
        return navAgent.targetPosition + gridOffset;
    }

    [BurstCompile]
    private void AssignGridPositions(NativeArray<Entity> agentEntities, ref SystemState state)
    {
        int unitCount = agentEntities.Length;
        float spacing = 0.8f; // Adjust for overall density of units
        float angleIncrement = math.radians(137.5f); // Golden angle for even distribution in spiral

        for (int i = 0; i < unitCount; i++)
        {
            Entity agentEntity = agentEntities[i];
            var navAgent = state.EntityManager.GetComponentData<NavAgentComponent>(agentEntity);

            // Calculate radius and angle for the current unit
            float radius = spacing * math.sqrt(i); // Expands gradually to space units evenly
            float angle = i * angleIncrement;

            // Calculate x and y offsets using the radius and angle
            float x = radius * math.cos(angle);
            float y = radius * math.sin(angle);

            // Store as grid position (or directly in a float3 if desired)
            navAgent.gridPosition = new int2((int)math.round(x), (int)math.round(y));
            state.EntityManager.SetComponentData(agentEntity, navAgent);
        }
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
        float avoidanceStrengthMultiplier = 5.0f; // Increase to make avoidance stronger

        // Random offset to avoid perfectly symmetrical avoidance
        float randomOffset = math.radians(10.0f) * (entity.Index % 2 == 0 ? 1 : -1); // Apply based on entity ID parity

        // Loop through all other agents
        for (int i = 0; i < agentEntities.Length; i++)
        {
            Entity otherEntity = agentEntities[i];

            // Skip self
            if (otherEntity == entity)
                continue;

            float3 otherPosition = agentPositions[i];
            float distance = math.distance(agentPosition, otherPosition);

            // Check if the other agent is within the avoidance radius
            if (distance < navAgent.ValueRO.avoidanceRadius && distance > 0)
            {
                float3 directionAway = agentPosition - otherPosition;
                float sqrDistance = math.lengthsq(directionAway);
                if (sqrDistance < math.pow(navAgent.ValueRO.avoidanceRadius, 2) && sqrDistance > 0)
                {
                    float3 normalizedDirectionAway = directionAway / math.sqrt(sqrDistance); // Only compute sqrt once
                    float weight = math.pow((navAgent.ValueRO.avoidanceRadius - math.sqrt(sqrDistance)) / navAgent.ValueRO.avoidanceRadius, 2);
                    avoidanceVelocity += normalizedDirectionAway * weight * avoidanceStrengthMultiplier;
                    neighborCount++;
                }
            }
        }

        // Normalize the avoidance velocity if there are neighbors contributing
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
    Entity entity,
    DynamicBuffer<WaypointBuffer> waypointBuffer)
    {
        navAgent.ValueRW.reachedDestination = false;

        NavMeshQuery query = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.TempJob, 1000);

        float3 fromPosition = transform.ValueRO.Position;

        // Calculate a unique target position for each agent using grid spacing
        float3 gridTargetPosition = CalculateGridTargetPosition(navAgent.ValueRO);
        float3 extents = new float3(10, 5, 10);

        // Map target to the nearest valid position on the NavMesh
        NavMeshLocation mappedTargetLocation = query.MapLocation(gridTargetPosition, extents, 0);
        float3 navMeshTargetPosition = mappedTargetLocation.position;

        // If no valid NavMesh location found, skip pathfinding
        if (!query.IsValid(mappedTargetLocation))
        {
            Debug.LogWarning($"No valid NavMesh location found for entity {entity.Index}");
            query.Dispose();
            return;
        }

        NavMeshLocation fromLocation = query.MapLocation(fromPosition, extents, 0);

        PathQueryStatus status;
        PathQueryStatus returningStatus;
        int maxPathSize = 500;

        if (query.IsValid(fromLocation) && query.IsValid(mappedTargetLocation))
        {
            status = query.BeginFindPath(fromLocation, mappedTargetLocation);
            if (status == PathQueryStatus.InProgress)
            {
                status = query.UpdateFindPath(1500, out int iterationsPerformed);
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
                        navMeshTargetPosition,
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

                        if (waypointBuffer.Length == 0)
                        {
                            Debug.LogWarning($"No waypoints generated for entity {entity.Index}");
                        }
                    }
                    else
                    {
                        Debug.LogWarning($"Path calculation failed for entity {entity.Index}");
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