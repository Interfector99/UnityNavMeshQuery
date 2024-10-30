using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Experimental.AI;

public struct WaypointRpcCommand : IRpcCommand
{
    public int GhostId;
    public float3 Waypoint;
}

[WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
[BurstCompile]
public partial struct ClientPathCalculationSystem : ISystem
{
    private NavMeshQuery navMeshQuery;
    private bool navMeshQueryInitialized;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<EntitiesReferences>();
        navMeshQueryInitialized = false;
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        if (!navMeshQueryInitialized)
        {
            var navMeshWorld = NavMeshWorld.GetDefaultWorld();
            int nodePoolSize = 2048;  // Adjust this as needed based on pathfinding requirements
            navMeshQuery = new NavMeshQuery(navMeshWorld, Allocator.Persistent, nodePoolSize);  // Specify the node pool size
            navMeshQueryInitialized = true;
        }

        var ecb = new EntityCommandBuffer(Allocator.TempJob);

        foreach (var (navAgent, transform, ghostInstance, entity)
                 in SystemAPI.Query<RefRW<NavAgentComponent>, RefRW<LocalTransform>, RefRO<GhostInstance>>().WithEntityAccess())
        {
            DynamicBuffer<WaypointBuffer> waypointBuffer = state.EntityManager.GetBuffer<WaypointBuffer>(entity);

            if (!navAgent.ValueRO.pathCalculated)
            {
                navAgent.ValueRW.pathCalculated = true;
                CalculatePath(navAgent, transform, entity, waypointBuffer);

                // Send waypoints to the server
                SendWaypointsToServer(waypointBuffer, ghostInstance.ValueRO.ghostId, ref ecb);
            }
        }

        ecb.Playback(state.EntityManager);
        ecb.Dispose();
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state)
    {
        if (navMeshQueryInitialized)
        {
            navMeshQuery.Dispose();
        }
    }

    [BurstCompile]
    private void SendWaypointsToServer(DynamicBuffer<WaypointBuffer> waypointBuffer, int ghostId, ref EntityCommandBuffer ecb)
    {
        foreach (var waypoint in waypointBuffer)
        {
            Entity rpcEntity = ecb.CreateEntity();
            ecb.AddComponent(rpcEntity, new WaypointRpcCommand
            {
                GhostId = ghostId,
                Waypoint = waypoint.wayPoint
            });
            ecb.AddComponent(rpcEntity, new SendRpcCommandRequest { TargetConnection = Entity.Null });
        }
    }

    [BurstCompile]
    private void CalculatePath(
    RefRW<NavAgentComponent> navAgent,
    RefRW<LocalTransform> transform,
    Entity entity,
    DynamicBuffer<WaypointBuffer> waypointBuffer)
    {
        Debug.Log($"Calculating path for {entity}");
        navAgent.ValueRW.reachedDestination = false;
        waypointBuffer.Clear();

        float3 fromPosition = transform.ValueRO.Position;
        float3 targetPosition = navAgent.ValueRO.targetPosition;
        float3 extents = new float3(10, 5, 10); // Adjust as needed to fit your NavMesh area

        NavMeshLocation fromLocation = navMeshQuery.MapLocation(fromPosition, extents, 0);
        NavMeshLocation toLocation = navMeshQuery.MapLocation(targetPosition, extents, 0);

        if (!navMeshQuery.IsValid(fromLocation) || !navMeshQuery.IsValid(toLocation))
        {
            Debug.LogWarning($"Invalid NavMesh locations: from {fromLocation.position} to {toLocation.position}");
            return;
        }

        // Begin path finding
        var pathStatus = navMeshQuery.BeginFindPath(fromLocation, toLocation);
        Debug.Log($"BeginFindPath status: {pathStatus}");
        if (pathStatus == PathQueryStatus.InProgress)
        {
            pathStatus = navMeshQuery.UpdateFindPath(1500, out int iterationsPerformed);
            Debug.Log($"UpdateFindPath status: {pathStatus}, Iterations: {iterationsPerformed}");

            pathStatus = navMeshQuery.EndFindPath(out int pathSize);
            Debug.Log($"EndFindPath status: {pathStatus}, Path size: {pathSize}");

            if (pathSize > 0)
            {
                Debug.Log($"Extracting waypoints, path size: {pathSize}");
                var polygonIds = new NativeArray<PolygonId>(pathSize, Allocator.Temp);
                int status = navMeshQuery.GetPathResult(polygonIds);

                // Get the straight path
                var result = new NativeArray<NavMeshLocation>(pathSize, Allocator.Temp);
                var straightPathFlags = new NativeArray<StraightPathFlags>(pathSize, Allocator.Temp);
                var vertexSide = new NativeArray<float>(pathSize, Allocator.Temp);

                int straightPathCount = 0;
                var straightPathStatus = PathUtils.FindStraightPath(
                    navMeshQuery,
                    fromPosition,
                    toLocation.position,
                    polygonIds,
                    pathSize,
                    ref result,
                    ref straightPathFlags,
                    ref vertexSide,
                    ref straightPathCount,
                    500
                );

                if (straightPathStatus == PathQueryStatus.Success || straightPathStatus == PathQueryStatus.PartialResult)
                {
                    Debug.Log($"Straight path count: {straightPathCount}");
                    for (int i = 0; i < straightPathCount; i++)
                    {
                        float3 waypoint = result[i].position;
                        waypointBuffer.Add(new WaypointBuffer { wayPoint = waypoint });
                        Debug.Log($"Waypoint added: {waypoint}");
                    }
                    navAgent.ValueRW.currentWaypoint = 0;
                }

                // Dispose of the temporary arrays
                result.Dispose();
                straightPathFlags.Dispose();
                vertexSide.Dispose();
                polygonIds.Dispose();
            }
            else
            {
                Debug.LogWarning("Pathfinding completed but no valid path could be extracted.");
            }
        }
    }
}
