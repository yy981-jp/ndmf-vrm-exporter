# VRMC_springBone_limit export

The exporter writes the optional `VRMC_springBone_limit` extension automatically for each exported PhysBone joint whose limit type is not `None`. No exporter setting is required.

## Mapping

| VRChat PhysBone limit | VRM extension shape | Values |
| --- | --- | --- |
| Angle | `cone` | `angle` from `maxAngleX` |
| Hinge | `hinge` | `angle` from `maxAngleX` |
| Polar | `spherical` | `pitch` from `maxAngleX`, `yaw` from `maxAngleZ` |
| None | omitted | No limit extension |

Angles are converted from degrees to radians. When the corresponding PhysBone angle curve is present, its value is evaluated at the joint's normalized depth before conversion. `limitRotation` is converted to the VRM coordinate space.

The extension is optional and currently uses `specVersion: 1.0-draft`. A viewer must implement `VRMC_springBone_limit` to apply these limits; viewers that do not support it will ignore the extension and continue using their normal SpringBone behavior.
