using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public struct NavAgentComponent : IComponentData
{
    public float3 targetPosition;
    public int currentWaypoint;
    public float moveSpeed;
    public bool pathCalculated;
}

public struct WaypointBuffer : IBufferElementData
{
    public float3 wayPoint;
}

public class NavAgentAuthoring : MonoBehaviour
{
    [SerializeField] private GameObject targetGameObject;
    [SerializeField] private float moveSpeed;

    private class Baker : Baker<NavAgentAuthoring>
    {
        public override void Bake(NavAgentAuthoring authoring)
        {
            Entity authoringEntity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(authoringEntity, new NavAgentComponent
            {
                targetPosition = authoring.targetGameObject.transform.position,
                moveSpeed = authoring.moveSpeed,
                pathCalculated = false
            });
            AddBuffer<WaypointBuffer>(authoringEntity);
        }
    }
}
