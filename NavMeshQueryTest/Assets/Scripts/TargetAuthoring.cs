using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public struct Target : IComponentData
{
    
}

public class TargetAuthoring : MonoBehaviour
{
    private class Baker : Baker<TargetAuthoring>
    {
        public override void Bake(TargetAuthoring authoring)
        {
            Entity authoringEntity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(authoringEntity, new Target
            {
                
            });
        }
    }
}
