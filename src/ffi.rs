//! Low-level C adapter. All calls require valid, live IDs, valid finite inputs,
//! properly sized pointers, and external serialization with every other Box3D
//! call (including calls made by the plugin). Prefer the safe ECS components.
#![allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}

unsafe extern "C" {
    pub fn bb_world_create(gravity: *const f32) -> u32;
    pub fn bb_world_destroy(world: u32);
    pub fn bb_world_step(world: u32, gravity: *const f32, dt: f32, substeps: i32);
    pub fn bb_body_create(world: u32, body_type: i32, state: *const State) -> u64;
    pub fn bb_body_destroy(body: u64);
    pub fn bb_body_set_type(body: u64, body_type: i32);
    pub fn bb_body_set_state(body: u64, state: *const State, flags: u32);
    pub fn bb_body_get_state(body: u64, state: *mut State);
    pub fn bb_body_apply(body: u64, force: *const f32, torque: *const f32, impulse: i32);
    pub fn bb_shape_create(body: u64, kind: i32, size: *const f32, material: *const f32) -> u64;
    pub fn bb_shape_destroy(shape: u64);
}
