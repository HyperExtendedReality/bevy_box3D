use bevy_app::{App, FixedUpdate};
use bevy_box3d::{Box3dPlugin, Collider, RigidBody};
use bevy_math::Vec3;
use bevy_time::{Fixed, Time};
use bevy_transform::components::Transform;

fn main() {
    let mut app = App::new();
    app.insert_resource(Time::<Fixed>::from_hz(60.0))
        .add_plugins(Box3dPlugin);
    app.world_mut().spawn((
        RigidBody::Static,
        Collider::Cuboid {
            half_extents: Vec3::new(10.0, 0.5, 10.0),
        },
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));
    let cube = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::Cuboid {
                half_extents: Vec3::splat(0.5),
            },
            Transform::from_xyz(0.0, 4.0, 0.0),
        ))
        .id();
    // Headless example: drive the fixed schedule directly instead of a game loop.
    for tick in 0..180 {
        app.world_mut().run_schedule(FixedUpdate);
        if tick % 30 == 0 {
            println!(
                "tick {tick}: {:?}",
                app.world().get::<Transform>(cube).unwrap().translation
            );
        }
    }
    let height = app.world().get::<Transform>(cube).unwrap().translation.y;
    assert!((height - 0.5).abs() < 0.08);
}
