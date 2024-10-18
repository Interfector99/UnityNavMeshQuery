using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public struct NavAgentComponent : IComponentData
{
    public Entity targetEntity;
    public bool pathCalculated;
    public int currentWaypoint;
    public float moveSpeed;
    public float nextPathCalculateTime;
    public float avoidanceRadius;
}

public struct WaypointBuffer : IBufferElementData
{
    public float3 wayPoint;
}

public class NavAgentAuthoring : MonoBehaviour
{
    [SerializeField] private Transform targetTransform;
    [SerializeField] private float moveSpeed;
    [SerializeField] private float avoidanceRadius;

    private class AuthoringBaker : Baker<NavAgentAuthoring>
    {
        public override void Bake(NavAgentAuthoring authoring)
        {
            Entity authoringEntity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(authoringEntity, new NavAgentComponent
            {
                targetEntity = GetEntity(authoring.targetTransform, TransformUsageFlags.Dynamic),
                moveSpeed = authoring.moveSpeed,
                avoidanceRadius = authoring.avoidanceRadius,
            });
            AddBuffer<WaypointBuffer>(authoringEntity);
        }
    }
}
