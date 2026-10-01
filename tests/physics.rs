use bevy_app::{App, FixedUpdate};
use bevy_box3d::*;
use bevy_math::{Quat, Vec3};
use bevy_time::{Fixed, Time, TimePlugin, TimeUpdateStrategy};
use bevy_transform::components::Transform;

fn app(gravity: Vec3) -> App {
    let mut app = App::new();
    app.insert_resource(Box3dSettings {
        gravity,
        ..Default::default()
    })
    .insert_resource(Time::<Fixed>::from_hz(60.0))
    .add_plugins(Box3dPlugin);
    app
}

fn step(app: &mut App, count: usize) {
    for _ in 0..count {
        app.world_mut().run_schedule(FixedUpdate);
    }
}

#[test]
fn colliders_fall_and_rest_on_ground() {
    let mut app = app(Vec3::new(0.0, -9.81, 0.0));
    let ground = app
        .world_mut()
        .spawn((
            RigidBody::Static,
            Collider::Cuboid {
                half_extents: Vec3::new(10.0, 0.5, 10.0),
            },
            Transform::from_xyz(0.0, -0.5, 0.0),
        ))
        .id();
    let shapes = [
        (
            Collider::Cuboid {
                half_extents: Vec3::splat(0.5),
            },
            0.5,
        ),
        (Collider::Sphere { radius: 0.5 }, 0.5),
        (
            Collider::Capsule {
                radius: 0.5,
                half_height: 0.5,
            },
            1.0,
        ),
    ];
    let bodies: Vec<_> = shapes
        .iter()
        .enumerate()
        .map(|(i, (shape, _))| {
            app.world_mut()
                .spawn((
                    RigidBody::Dynamic,
                    *shape,
                    Transform::from_xyz(i as f32 * 3.0 - 3.0, 4.0, 0.0),
                ))
                .id()
        })
        .collect();
    step(&mut app, 30);
    for entity in &bodies {
        assert!(app.world().get::<Transform>(*entity).unwrap().translation.y < 3.0);
    }
    step(&mut app, 270);
    for (entity, (_, height)) in bodies.iter().zip(shapes) {
        let transform = app.world().get::<Transform>(*entity).unwrap();
        assert!(
            (transform.translation.y - height).abs() < 0.08,
            "unexpected resting position: {transform:?}"
        );
        assert!(
            app.world()
                .get::<Velocity>(*entity)
                .unwrap()
                .linear
                .length()
                < 0.1
        );
    }
    assert_eq!(
        app.world().get::<Transform>(ground).unwrap().translation.y,
        -0.5
    );
    assert_eq!(app.world().resource::<PhysicsWorld>().body_count(), 4);
}

#[test]
fn edits_impulses_and_removal_reconcile_native_bodies() {
    let mut app = app(Vec3::ZERO);
    let entity = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::Sphere { radius: 0.5 },
            PhysicsMaterial {
                density: 1.0,
                ..Default::default()
            },
            ExternalImpulse {
                linear: Vec3::X,
                ..Default::default()
            },
        ))
        .id();
    step(&mut app, 1);
    assert_eq!(
        app.world().get::<ExternalImpulse>(entity).unwrap().linear,
        Vec3::ZERO
    );
    let speed = app.world().get::<Velocity>(entity).unwrap().linear.x;
    assert!(speed > 1.0);
    step(&mut app, 1);
    assert!((app.world().get::<Velocity>(entity).unwrap().linear.x - speed).abs() < 0.001);

    let rotation = Quat::from_rotation_z(0.3);
    app.world_mut().entity_mut(entity).insert((
        Transform {
            translation: Vec3::new(10.0, 5.0, 0.0),
            rotation,
            scale: Vec3::splat(2.0),
        },
        Velocity {
            linear: Vec3::new(2.0, 0.0, 0.0),
            ..Default::default()
        },
        Collider::Cuboid {
            half_extents: Vec3::splat(0.75),
        },
    ));
    step(&mut app, 1);
    let transform = app.world().get::<Transform>(entity).unwrap();
    assert!((transform.translation.x - (10.0 + 2.0 / 60.0)).abs() < 0.001);
    assert!((transform.rotation.dot(rotation).abs() - 1.0).abs() < 0.001);
    assert_eq!(transform.scale, Vec3::splat(2.0));

    app.world_mut().entity_mut(entity).remove::<Collider>();
    step(&mut app, 1);
    assert!(!app.world().resource::<PhysicsWorld>().contains(entity));
    app.world_mut()
        .entity_mut(entity)
        .insert(Collider::Sphere { radius: 1.0 });
    step(&mut app, 1);
    assert!(app.world().resource::<PhysicsWorld>().contains(entity));
    app.world_mut().entity_mut(entity).remove::<RigidBody>();
    step(&mut app, 1);
    assert_eq!(app.world().resource::<PhysicsWorld>().body_count(), 0);
    app.world_mut()
        .entity_mut(entity)
        .insert(RigidBody::Kinematic);
    step(&mut app, 1);
    app.world_mut().despawn(entity);
    step(&mut app, 1);
    assert_eq!(app.world().resource::<PhysicsWorld>().body_count(), 0);
}

#[test]
fn kinematic_motion_force_and_body_type_changes() {
    let mut app = app(Vec3::ZERO);
    let kinematic = app
        .world_mut()
        .spawn((
            RigidBody::Kinematic,
            Collider::Sphere { radius: 0.5 },
            Velocity {
                linear: Vec3::X,
                ..Default::default()
            },
        ))
        .id();
    let dynamic = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::Sphere { radius: 0.5 },
            Transform::from_xyz(0.0, 10.0, 0.0),
            PhysicsMaterial {
                density: 1.0,
                ..Default::default()
            },
            ExternalForce {
                force: Vec3::X,
                torque: Vec3::Z,
            },
        ))
        .id();
    step(&mut app, 60);
    assert!(
        (app.world()
            .get::<Transform>(kinematic)
            .unwrap()
            .translation
            .x
            - 1.0)
            .abs()
            < 0.001
    );
    let velocity = app.world().get::<Velocity>(dynamic).unwrap();
    assert!(velocity.linear.x > 1.0 && velocity.angular.z > 1.0);
    app.world_mut()
        .entity_mut(dynamic)
        .insert(RigidBody::Static);
    step(&mut app, 1);
    let position = app.world().get::<Transform>(dynamic).unwrap().translation;
    step(&mut app, 10);
    assert_eq!(
        app.world().get::<Transform>(dynamic).unwrap().translation,
        position
    );
    app.world_mut()
        .entity_mut(dynamic)
        .insert(RigidBody::Dynamic);
    step(&mut app, 10);
    assert!(app.world().get::<Transform>(dynamic).unwrap().translation.x > position.x);
}

#[test]
#[should_panic(expected = "sphere radius must be finite and positive")]
fn invalid_shape_is_rejected_before_ffi() {
    let mut app = app(Vec3::ZERO);
    app.world_mut()
        .spawn((RigidBody::Dynamic, Collider::Sphere { radius: f32::NAN }));
    step(&mut app, 1);
}

#[test]
fn time_plugin_drives_physics_and_world_replacement_rebuilds_bodies() {
    let mut app = app(Vec3::ZERO);
    app.add_plugins(TimePlugin)
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f64(1.0 / 60.0),
        ));
    let entity = app
        .world_mut()
        .spawn((
            RigidBody::Kinematic,
            Collider::Sphere { radius: 0.5 },
            Velocity {
                linear: Vec3::X,
                ..Default::default()
            },
        ))
        .id();
    for _ in 0..31 {
        app.update();
    }
    let x = app.world().get::<Transform>(entity).unwrap().translation.x;
    assert!(
        (0.4..0.6).contains(&x),
        "fixed schedule did not advance: {x}"
    );
    app.insert_resource(PhysicsWorld::default());
    assert_eq!(app.world().resource::<PhysicsWorld>().body_count(), 0);
    app.update();
    assert!(app.world().resource::<PhysicsWorld>().contains(entity));
    assert!(app.world().get::<Transform>(entity).unwrap().translation.x > x);
}
