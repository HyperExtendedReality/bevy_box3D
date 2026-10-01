//! Box3D rigid bodies for Bevy. See the crate README for setup and supported scope.

pub mod ffi;

use bevy_app::{App, FixedUpdate, Plugin};
use bevy_ecs::prelude::*;
use bevy_math::{Quat, Vec3};
use bevy_time::{Fixed, Time};
use bevy_transform::components::Transform;
use std::{collections::HashMap, sync::Mutex};

// ponytail: serialize native calls across worlds; use per-world locks once
// Box3D's global creation/allocator state supports concurrent access.
static NATIVE_LOCK: Mutex<()> = Mutex::new(());

/// Adds physics to `FixedUpdate`. Requires Bevy's `TimePlugin` (included in
/// `DefaultPlugins` and `MinimalPlugins`) to drive the fixed schedule.
pub struct Box3dPlugin;

impl Plugin for Box3dPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Box3dSettings>()
            .init_resource::<PhysicsWorld>()
            .init_resource::<Time<Fixed>>()
            .add_systems(FixedUpdate, simulate.in_set(Box3dSet));
    }
}

/// Order gameplay systems before this set to change physics inputs, or after it
/// to read the resulting transforms and velocities.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Box3dSet;

/// Runtime world settings. The timestep comes from Bevy's `Time<Fixed>`.
#[derive(Resource, Debug, Clone)]
pub struct Box3dSettings {
    pub gravity: Vec3,
    pub substeps: u32,
}

impl Default for Box3dSettings {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            substeps: 4,
        }
    }
}

/// Attach with a `Collider`. Physics entities must have no `ChildOf`: their
/// `Transform` is in world space. Scale is visual only; size the collider itself.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Default)]
#[require(Transform, Velocity, PhysicsMaterial)]
#[repr(i32)]
pub enum RigidBody {
    #[default]
    Static = 0,
    Kinematic = 1,
    Dynamic = 2,
}

/// One centered collider per body, in meters. Changes replace the native shape
/// without replacing the body or losing its velocity.
#[derive(Component, Debug, Clone, Copy, PartialEq)]
pub enum Collider {
    Cuboid {
        half_extents: Vec3,
    },
    Sphere {
        radius: f32,
    },
    /// Y-aligned capsule. `half_height` is half the distance between cap centers.
    Capsule {
        radius: f32,
        half_height: f32,
    },
}

impl Collider {
    fn native(self) -> (i32, [f32; 3]) {
        match self {
            Self::Cuboid { half_extents } => {
                assert!(
                    half_extents.is_finite() && half_extents.min_element() > 0.0,
                    "cuboid half extents must be finite and positive"
                );
                (0, half_extents.to_array())
            }
            Self::Sphere { radius } => {
                assert!(
                    radius.is_finite() && radius > 0.0,
                    "sphere radius must be finite and positive"
                );
                (1, [radius, 0.0, 0.0])
            }
            Self::Capsule {
                radius,
                half_height,
            } => {
                assert!(
                    radius.is_finite()
                        && radius > 0.0
                        && half_height.is_finite()
                        && half_height > 0.0,
                    "capsule radius and half height must be finite and positive"
                );
                (2, [radius, half_height, 0.0])
            }
        }
    }
}

/// Density in kg/m³, friction, and restitution. Density must be positive;
/// friction nonnegative; restitution in [0, 1]. All must be finite.
#[derive(Component, Debug, Clone, Copy, PartialEq)]
pub struct PhysicsMaterial {
    pub density: f32,
    pub friction: f32,
    pub restitution: f32,
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            density: 1000.0,
            friction: 0.6,
            restitution: 0.0,
        }
    }
}

impl PhysicsMaterial {
    fn native(self) -> [f32; 3] {
        assert!(
            self.density.is_finite() && self.density > 0.0,
            "density must be finite and positive"
        );
        assert!(
            self.friction.is_finite() && self.friction >= 0.0,
            "friction must be finite and nonnegative"
        );
        assert!(
            self.restitution.is_finite() && (0.0..=1.0).contains(&self.restitution),
            "restitution must be in [0, 1]"
        );
        [self.density, self.friction, self.restitution]
    }
}

/// World-space linear velocity (m/s) and angular velocity (rad/s). Changes are
/// sent to Box3D before stepping, then updated from the simulation afterward.
#[derive(Component, Debug, Clone, Copy, Default, PartialEq)]
pub struct Velocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

/// Persistent force (N) at the center of mass and torque (N·m), in world space.
/// Remove or zero this component to stop applying it.
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct ExternalForce {
    pub force: Vec3,
    pub torque: Vec3,
}

/// One-shot linear (N·s) and angular (N·m·s) impulses, in world space.
/// Automatically zeroed after applying them.
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct ExternalImpulse {
    pub linear: Vec3,
    pub angular: Vec3,
}

/// Owns the native world and the entity-to-body mapping. Dropping this resource
/// frees all native bodies and shapes. Handles stay private to prevent stale IDs.
#[derive(Resource)]
pub struct PhysicsWorld {
    id: u32,
    bodies: HashMap<Entity, NativeBody>,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        let _guard = NATIVE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // SAFETY: serialized creation, finite gravity array valid for the call.
        let id = unsafe { ffi::bb_world_create([0.0, -9.81, 0.0].as_ptr()) };
        assert_ne!(id, 0, "Box3D could not create a world");
        Self {
            id,
            bodies: HashMap::new(),
        }
    }
}

impl PhysicsWorld {
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
    pub fn contains(&self, entity: Entity) -> bool {
        self.bodies.contains_key(&entity)
    }
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        let _guard = NATIVE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // SAFETY: this resource exclusively owns this live world.
        unsafe { ffi::bb_world_destroy(self.id) };
    }
}

struct NativeBody {
    id: u64,
    shape: u64,
    body_type: RigidBody,
    collider: Collider,
    material: PhysicsMaterial,
    state: ffi::State,
}

fn state(transform: &Transform, velocity: &Velocity) -> ffi::State {
    assert!(
        transform.translation.is_finite()
            && transform.rotation.is_finite()
            && (transform.rotation.length_squared() - 1.0).abs() < 0.001,
        "physics position must be finite and rotation must be a unit quaternion"
    );
    assert!(
        velocity.linear.is_finite() && velocity.angular.is_finite(),
        "physics velocities must be finite"
    );
    ffi::State {
        position: transform.translation.to_array(),
        rotation: transform.rotation.to_array(),
        linear_velocity: velocity.linear.to_array(),
        angular_velocity: velocity.angular.to_array(),
    }
}

type PhysicsQuery<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static RigidBody,
        &'static Collider,
        &'static PhysicsMaterial,
        &'static mut Transform,
        &'static mut Velocity,
        Option<&'static ExternalForce>,
        Option<&'static mut ExternalImpulse>,
        Option<&'static ChildOf>,
    ),
>;

fn simulate(
    mut physics: ResMut<PhysicsWorld>,
    settings: Res<Box3dSettings>,
    time: Res<Time<Fixed>>,
    mut query: PhysicsQuery,
) {
    assert!(settings.gravity.is_finite(), "gravity must be finite");
    assert!(
        settings.substeps > 0 && settings.substeps <= i32::MAX as u32,
        "substeps must fit a positive C int"
    );
    let dt = time.timestep().as_secs_f32();
    assert!(
        dt.is_finite() && dt > 0.0,
        "physics timestep must be finite and positive"
    );
    // Validate every entity before changing native state or consuming impulses.
    for (_, _, collider, material, transform, velocity, force, impulse, parent) in &mut query {
        assert!(
            parent.is_none(),
            "Box3D rigid bodies must be unparented; Transform is world space"
        );
        collider.native();
        material.native();
        state(&transform, &velocity);
        if let Some(force) = force {
            assert!(
                force.force.is_finite() && force.torque.is_finite(),
                "force and torque must be finite"
            );
        }
        if let Some(impulse) = impulse {
            assert!(
                impulse.linear.is_finite() && impulse.angular.is_finite(),
                "impulses must be finite"
            );
        }
    }

    let _guard = NATIVE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: all IDs belong to this privately owned live world, the global lock
    // serializes native calls, input data was validated, and pointer arguments
    // reference live arrays/structs for each call. Box3D copies shape definitions.
    unsafe {
        physics.bodies.retain(|entity, body| {
            if query.get(*entity).is_ok() {
                return true;
            }
            ffi::bb_body_destroy(body.id);
            false
        });
        let world_id = physics.id;
        for (entity, body_type, collider, material, transform, velocity, force, impulse, _) in
            &mut query
        {
            let input = state(&transform, &velocity);
            let (kind, size) = collider.native();
            let material_values = material.native();
            let body = physics.bodies.entry(entity).or_insert_with(|| {
                let id = ffi::bb_body_create(world_id, *body_type as i32, &input);
                assert_ne!(id, 0, "Box3D could not create a body");
                NativeBody {
                    id,
                    shape: 0,
                    body_type: *body_type,
                    collider: *collider,
                    material: *material,
                    state: input,
                }
            });
            if body.body_type != *body_type {
                ffi::bb_body_set_type(body.id, *body_type as i32);
                body.body_type = *body_type;
            }
            if body.shape == 0 || body.collider != *collider || body.material != *material {
                if body.shape != 0 {
                    ffi::bb_shape_destroy(body.shape);
                }
                body.shape =
                    ffi::bb_shape_create(body.id, kind, size.as_ptr(), material_values.as_ptr());
                assert_ne!(body.shape, 0, "Box3D could not create a collider");
                body.collider = *collider;
                body.material = *material;
            }
            let mut flags = 0;
            if input.position != body.state.position || input.rotation != body.state.rotation {
                flags |= 1;
            }
            if input.linear_velocity != body.state.linear_velocity {
                flags |= 2;
            }
            if input.angular_velocity != body.state.angular_velocity {
                flags |= 4;
            }
            if flags != 0 {
                ffi::bb_body_set_state(body.id, &input, flags);
            }
            if let Some(force) = force
                && (force.force != Vec3::ZERO || force.torque != Vec3::ZERO)
            {
                ffi::bb_body_apply(
                    body.id,
                    force.force.to_array().as_ptr(),
                    force.torque.to_array().as_ptr(),
                    0,
                );
            }
            if let Some(mut impulse) = impulse
                && (impulse.linear != Vec3::ZERO || impulse.angular != Vec3::ZERO)
            {
                ffi::bb_body_apply(
                    body.id,
                    impulse.linear.to_array().as_ptr(),
                    impulse.angular.to_array().as_ptr(),
                    1,
                );
                *impulse = ExternalImpulse::default();
            }
        }
        ffi::bb_world_step(
            physics.id,
            settings.gravity.to_array().as_ptr(),
            dt,
            settings.substeps as i32,
        );
        for (entity, _, _, _, mut transform, mut velocity, _, _, _) in &mut query {
            let body = physics
                .bodies
                .get_mut(&entity)
                .expect("body synchronized above");
            ffi::bb_body_get_state(body.id, &mut body.state);
            let position = Vec3::from_array(body.state.position);
            let rotation = Quat::from_array(body.state.rotation);
            if transform.translation != position || transform.rotation != rotation {
                transform.translation = position;
                transform.rotation = rotation;
            }
            let output = Velocity {
                linear: Vec3::from_array(body.state.linear_velocity),
                angular: Vec3::from_array(body.state.angular_velocity),
            };
            if *velocity != output {
                *velocity = output;
            }
        }
    }
}
