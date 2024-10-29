using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public struct NavAgentComponent : IComponentData
{
    public float3 targetPosition;
    public int currentWaypoint;
    public float moveSpeed;
    public bool pathCalculated;

    public float avoidanceRadius;
    public int2 gridPosition;

    public bool reachedDestination;
}


public struct WaypointBuffer : IBufferElementData
{
    public float3 wayPoint;
}

public class NavAgentAuthoring : MonoBehaviour
{
    [SerializeField] private GameObject targetGameObject;
    [SerializeField] private float moveSpeed;

    [SerializeField] private float avoidanceRadius;

    private class Baker : Baker<NavAgentAuthoring>
    {
        public override void Bake(NavAgentAuthoring authoring)
        {
            Entity authoringEntity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(authoringEntity, new NavAgentComponent
            {
                targetPosition = authoring.targetGameObject.transform.position,
                moveSpeed = authoring.moveSpeed,
                pathCalculated = false,

                avoidanceRadius = authoring.avoidanceRadius
            });
            AddBuffer<WaypointBuffer>(authoringEntity);
        }
    }
}
