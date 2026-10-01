# bevy_box3d

Box3D's native C17 engine, statically linked into a standalone Bevy ECS plugin.
This checkout uses the sibling Bevy **0.20.0-dev** crates in `../../bevy` so
components share the same Bevy types as applications using that checkout.
Use the same Bevy source in your application; a crates.io Bevy dependency has
different types even if its version looks similar.

## Build and run

Requires Rust, a C17 compiler (MSVC Build Tools on Windows, GCC/Clang on Unix),
and the sibling `../box3d` source checkout. Cargo compiles the native library;
CMake, bindgen, libclang, sample dependencies, and runtime DLLs are unnecessary.
Set `BOX3D_SOURCE_DIR` to an absolute path to use a different Box3D checkout
compatible with this adapter. Box3D is built in single precision.

```sh
cargo test --manifest-path bevy_box3d/Cargo.toml
cargo run --manifest-path bevy_box3d/Cargo.toml --example falling_box
```

In an application using the same local Bevy checkout:

```toml
[dependencies]
bevy_box3d = { path = "../crates/bevy_box3d" }
```

```rust,ignore
use bevy::prelude::*;
use bevy_box3d::{Box3dPlugin, Collider, RigidBody};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, Box3dPlugin))
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn((
        RigidBody::Static,
        Collider::Cuboid { half_extents: Vec3::new(10.0, 0.5, 10.0) },
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));
    commands.spawn((
        RigidBody::Dynamic,
        Collider::Cuboid { half_extents: Vec3::splat(0.5) },
        Transform::from_xyz(0.0, 4.0, 0.0),
    ));
}
```

## Behavior

- `RigidBody` requires `Transform`, `Velocity`, and `PhysicsMaterial`, inserting
  defaults when absent. A body is simulated once it also has a `Collider`.
- `Box3dSettings` controls gravity and solver substeps (default: 4). The step
  duration follows `Time<Fixed>`; the plugin preserves your existing timestep.
  A normal game must install Bevy's `TimePlugin`, included in `DefaultPlugins`
  and `MinimalPlugins`. The example drives `FixedUpdate` manually.
- Changes to position, rotation, velocity, body type, collider, or material
  synchronize before each step. Dynamic and kinematic results write back after
  stepping. Direct transform edits teleport and wake the body.
- `ExternalForce` persists each step. `ExternalImpulse` applies once and is
  zeroed. Zero forces/impulses do not keep sleeping bodies awake.
- Put gameplay input systems `.before(Box3dSet)` and result readers
  `.after(Box3dSet)` within `FixedUpdate`.
- Removing an entity, `RigidBody`, `Collider`, or a required component destroys
  its native body at the next fixed step. Dropping `PhysicsWorld` frees the
  native world immediately. Replacing it rebuilds bodies on the next step.
- Coordinates use meters, kilograms, seconds, and unit quaternions. Entities
  must be unparented: `Transform` is world space. Transform scale is visual
  only; set collider dimensions explicitly. Invalid input panics in Rust
  before entering the native engine.
- Each entity supports one centered cuboid, sphere, or Y-aligned capsule.
  Compound shapes, meshes, height fields, joints, sensors, collision events,
  spatial queries, interpolation, and debug rendering are not wrapped yet.
- The native engine uses one worker. Native calls are serialized across worlds
  because Box3D has shared global state. No unsafe `Send`/`Sync` implementations
  or user-visible native handles are needed.

`ffi` exposes the unsafe C adapter for low-level use. Its integer handles are
opaque; callers must manage lifetimes and serialize all native calls. Safe
applications should use the ECS interface. The adapter compiles against
Box3D's headers, keeping its definition structs and inline ID helpers in C.

This crate is not configured for crates.io publishing because it depends on
local Bevy and Box3D checkouts. Box3D retains its upstream MIT license in
`../box3d/LICENSE`.
