# Field JSON schema — `frcsim.field/1`

A field file describes static geometry, materials, bounds, piece types, and initial piece placements.
It is parsed natively (`native/src/field/field_json.cpp`) so Java, C++ tests, and benchmarks share one
loader. Example: [`fields/test-flat/field.json`](../../fields/test-flat/field.json).

Conventions: SI units (meters, kilograms), WPILib field frame (origin at blue alliance wall corner,
+X toward red, +Y left, +Z up). Rotations are `rpyDeg: [roll, pitch, yaw]` in degrees, applied as
`Rz(yaw)·Ry(pitch)·Rx(roll)`, the same convention as WPILib `Rotation3d`.

Loading is **not atomic**: on error the world may be partially populated. Create a fresh world to retry.

## Top level

| key | type | required | description |
|---|---|---|---|
| `schema` | string | yes | must be `"frcsim.field/1"` |
| `name` | string | no | informational |
| `materials` | object | no | `name → {friction, restitution}`; adds or updates materials |
| `materialPairs` | array | no | `{a, b, friction, restitution}` pair overrides |
| `ground` | object | no | `{material, height}`: large slab whose top surface is at `height` (default 0) |
| `statics` | array | no | static primitives (below) |
| `bounds` | object | no | `{min: [x,y,z], max: [x,y,z]}`; pieces leaving it become `OutOfBounds` |
| `pieceTypes` | object | no | `name → piece type` (below) |
| `pieceSpawns` | array | no | initial placements (below) |

Materials referenced by name must be defined, either earlier in the same file or already in the world
(the world starts with `default` plus the standard FRC materials).

## Static primitives

Common keys: `type`, `name` (optional), `material` (default `"default"`), `center` `[x,y,z]`, `rpyDeg`
(optional).

| `type` | extra keys |
|---|---|
| `box` | `halfExtents` `[x,y,z]` |
| `cylinder` | `radius`, `halfHeight` (axis along local Z) |
| `convexHull` | `points` `[[x,y,z], …]` (≥ 4 non-coplanar, relative to `center`) |

## Piece types

| key | type | default | description |
|---|---|---|---|
| `shape` | `"sphere"` \| `"cylinder"` \| `"box"` | — | |
| `radius` | number | — | sphere, cylinder |
| `halfHeight` | number | — | cylinder (axis along Z) |
| `halfExtents` | `[x,y,z]` | — | box |
| `mass` | number | — | kg |
| `material` | string | `"default"` | |
| `maxAngularVelocity` | number | 500 | rad/s (D14) |

## Piece spawns

Each entry has `type` plus exactly one of:
- `positions`: `[[x,y,z], …]`
- `grid`: `{origin: [x,y,z], count: [nx,ny,nz], spacing: [dx,dy,dz]}` places `nx·ny·nz` pieces at
  `origin + (i·dx, j·dy, k·dz)`.
