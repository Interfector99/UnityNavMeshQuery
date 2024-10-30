using UnityEngine;
using Unity.Entities;

public struct EntitiesReferences : IComponentData
{
    public Entity followerPrefab;
    public Entity targetPrefab;
}

public class EntitiesReferencesAuthoring : MonoBehaviour
{
    public GameObject followerSpawnerPrefabGameObject;
    public GameObject targetPrefabGameObject;

    public class Baker : Baker<EntitiesReferencesAuthoring>
    {
        public override void Bake(EntitiesReferencesAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new EntitiesReferences
            {
                followerPrefab  = GetEntity(authoring.followerSpawnerPrefabGameObject,  TransformUsageFlags.Dynamic),
                targetPrefab    = GetEntity(authoring.targetPrefabGameObject,           TransformUsageFlags.Dynamic),
            });
        }
    }
}
