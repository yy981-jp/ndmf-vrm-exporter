# PhysBone Global Collision export

The exporter includes avatar-scoped VRChat Global PhysBone Colliders in the VRM SpringBone ColliderGroups that can use them.

## Rules

- Global Collision support is enabled for VRC SDK 3.10.4 and newer.
- Only Sphere and Capsule `VRCPhysBoneCollider` components can be global. Plane colliders remain available only through explicit PhysBone collider lists.
- A global collider is exported only when its Global Collision scope includes Avatar.
- A PhysBone with Allow Collision set to True receives avatar-scoped global colliders.
- A PhysBone with Allow Collision set to False receives only its explicit Colliders list.
- A PhysBone with Allow Collision set to Other receives global colliders only when its permission filter allows self-collision and its content types include Avatar.
- Explicit colliders remain assigned regardless of Allow Collision, and colliders already present explicitly are not duplicated when they are also global.
- Excluded colliders, inactive colliders, and colliders whose root transform is not exported are not added to groups.

VRM SpringBone does not retain VRChat's global-collider concept as a runtime-wide setting. The exporter therefore expands the applicable avatar-global colliders into each affected Spring's ColliderGroup. Global collision against other avatars or worlds is outside a single-avatar VRM export.

The SDK 3.10.4 metadata is accessed reflectively so the exporter can still compile against older VRC SDK versions. If the SDK metadata cannot be read, the exporter fails closed and does not infer an avatar-global collider from an unknown flag value.
