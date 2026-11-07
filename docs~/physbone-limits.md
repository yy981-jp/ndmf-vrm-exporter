# VRMC_springBone_limit export

The exporter writes the optional `VRMC_springBone_limit` 1.0 extension automatically for each exported non-terminal PhysBone joint whose limit type is not `None`. The final joint in each chain is only a tail node and MUST NOT contain the extension. No exporter setting is required.

## Mapping

| VRChat PhysBone limit | VRM extension shape | Values |
| --- | --- | --- |
| Angle | `cone` | `angle` from `maxAngleX` |
| Hinge | `hinge` | `angle` from `maxAngleX` |
| Polar | `spherical` | `pitch` from `maxAngleX`, `yaw` from `maxAngleZ` |
| None | omitted | No limit extension |

Angles are converted from degrees to radians. When the corresponding PhysBone angle curve is present, its value is evaluated at the joint's normalized depth before conversion. `limitRotation` is converted to the VRM coordinate space.

The extension is optional. A viewer must implement `VRMC_springBone_limit` 1.0 to apply these limits; viewers that do not support it will ignore the extension and continue using their normal SpringBone behavior.
