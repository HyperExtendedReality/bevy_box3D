#include "box3d/box3d.h"

#include <string.h>

// Only fixed-width integers and float arrays cross the FFI boundary.
typedef struct bbState {
    float position[3];
    float rotation[4];
    float linear_velocity[3];
    float angular_velocity[3];
} bbState;

static b3Vec3 vec(const float v[3]) { return (b3Vec3){v[0], v[1], v[2]}; }
static b3Quat quat(const float q[4]) { return (b3Quat){{q[0], q[1], q[2]}, q[3]}; }

uint32_t bb_world_create(const float gravity[3]) {
    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = vec(gravity);
    def.workerCount = 1;
    return b3StoreWorldId(b3CreateWorld(&def));
}

void bb_world_destroy(uint32_t world) { b3DestroyWorld(b3LoadWorldId(world)); }

void bb_world_step(uint32_t world, const float gravity[3], float dt, int32_t substeps) {
    b3WorldId id = b3LoadWorldId(world);
    b3World_SetGravity(id, vec(gravity));
    b3World_Step(id, dt, substeps);
}

// Bodies Box3D moved in the last step (awake, or just fallen asleep). Writes up to
// capacity ids and returns the full count; the events last until the next step.
int32_t bb_world_moved(uint32_t world, uint64_t* bodies, int32_t capacity) {
    b3BodyEvents events = b3World_GetBodyEvents(b3LoadWorldId(world));
    for (int32_t i = 0; i < events.moveCount && i < capacity; ++i) {
        bodies[i] = b3StoreBodyId(events.moveEvents[i].bodyId);
    }
    return events.moveCount;
}

uint64_t bb_body_create(uint32_t world, int32_t type, const bbState* state) {
    b3BodyDef def = b3DefaultBodyDef();
    def.type = (b3BodyType)type;
    def.position = (b3Pos){state->position[0], state->position[1], state->position[2]};
    def.rotation = quat(state->rotation);
    def.linearVelocity = vec(state->linear_velocity);
    def.angularVelocity = vec(state->angular_velocity);
    return b3StoreBodyId(b3CreateBody(b3LoadWorldId(world), &def));
}

void bb_body_destroy(uint64_t body) { b3DestroyBody(b3LoadBodyId(body)); }
void bb_body_set_type(uint64_t body, int32_t type) { b3Body_SetType(b3LoadBodyId(body), (b3BodyType)type); }

void bb_body_set_state(uint64_t body, const bbState* state, uint32_t flags) {
    b3BodyId id = b3LoadBodyId(body);
    if (flags & 1) {
        b3Body_SetTransform(id, (b3Pos){state->position[0], state->position[1], state->position[2]}, quat(state->rotation));
        b3Body_SetAwake(id, true);
    }
    if (flags & 2) b3Body_SetLinearVelocity(id, vec(state->linear_velocity));
    if (flags & 4) b3Body_SetAngularVelocity(id, vec(state->angular_velocity));
}

void bb_body_get_state(uint64_t body, bbState* state) {
    b3BodyId id = b3LoadBodyId(body);
    b3Pos p = b3Body_GetPosition(id);
    b3Quat q = b3Body_GetRotation(id);
    b3Vec3 v = b3Body_GetLinearVelocity(id);
    b3Vec3 w = b3Body_GetAngularVelocity(id);
    *state = (bbState){{p.x, p.y, p.z}, {q.v.x, q.v.y, q.v.z, q.s}, {v.x, v.y, v.z}, {w.x, w.y, w.z}};
}

void bb_body_apply(uint64_t body, const float force[3], const float torque[3], int32_t impulse) {
    b3BodyId id = b3LoadBodyId(body);
    if (impulse) {
        b3Body_ApplyLinearImpulseToCenter(id, vec(force), true);
        b3Body_ApplyAngularImpulse(id, vec(torque), true);
    } else {
        b3Body_ApplyForceToCenter(id, vec(force), true);
        b3Body_ApplyTorque(id, vec(torque), true);
    }
}

uint64_t bb_shape_create(uint64_t body, int32_t kind, const float size[3], const float material[3]) {
    b3BodyId id = b3LoadBodyId(body);
    b3ShapeDef def = b3DefaultShapeDef();
    def.density = material[0];
    def.baseMaterial.friction = material[1];
    def.baseMaterial.restitution = material[2];
    b3ShapeId shape = {0};
    switch (kind) {
        case 0: {
            b3BoxHull hull = b3MakeBoxHull(size[0], size[1], size[2]);
            shape = b3CreateHullShape(id, &def, &hull.base);
            break;
        }
        case 1: {
            b3Sphere sphere = {{0, 0, 0}, size[0]};
            shape = b3CreateSphereShape(id, &def, &sphere);
            break;
        }
        case 2: {
            b3Capsule capsule = {{0, -size[1], 0}, {0, size[1], 0}, size[0]};
            shape = b3CreateCapsuleShape(id, &def, &capsule);
            break;
        }
    }
    return b3StoreShapeId(shape);
}

void bb_shape_destroy(uint64_t shape) { b3DestroyShape(b3LoadShapeId(shape), true); }

// Convex hulls and triangle meshes are created once and shared by every body
// using them. A hull shape clones its hull; a mesh shape keeps a reference, so
// a mesh must outlive its shapes. NULL when the input is degenerate.
void* bb_hull_create(const float* points, int32_t count) {
    return b3CreateHull((const b3Vec3*)points, count, B3_MAX_HULL_VERTICES);
}

void bb_hull_destroy(void* hull) { b3DestroyHull((b3HullData*)hull); }

// Face planes (normal, offset), up to capacity; returns the face count.
int32_t bb_hull_planes(const void* hull, float* planes, int32_t capacity) {
    const b3HullData* h = (const b3HullData*)hull;
    const b3Plane* p = b3GetHullPlanes(h);
    for (int32_t i = 0; i < h->faceCount && i < capacity; ++i) {
        planes[4 * i + 0] = p[i].normal.x;
        planes[4 * i + 1] = p[i].normal.y;
        planes[4 * i + 2] = p[i].normal.z;
        planes[4 * i + 3] = p[i].offset;
    }
    return h->faceCount;
}

void bb_hull_bounds(const void* hull, float lower[3], float upper[3]) {
    b3AABB box = ((const b3HullData*)hull)->aabb;
    lower[0] = box.lowerBound.x, lower[1] = box.lowerBound.y, lower[2] = box.lowerBound.z;
    upper[0] = box.upperBound.x, upper[1] = box.upperBound.y, upper[2] = box.upperBound.z;
}

void* bb_mesh_create(const float* vertices, int32_t vertex_count, const int32_t* indices, int32_t triangle_count) {
    b3MeshDef def = {0};
    def.vertices = (b3Vec3*)vertices;
    def.indices = (int32_t*)indices;
    def.vertexCount = vertex_count;
    def.triangleCount = triangle_count;
    def.identifyEdges = true;
    return b3CreateMesh(&def, NULL, 0);
}

void bb_mesh_destroy(void* mesh) { b3DestroyMesh((b3MeshData*)mesh); }

void bb_mesh_counts(const void* mesh, int32_t* nodes, int32_t* triangles) {
    const b3MeshData* m = (const b3MeshData*)mesh;
    *nodes = m->nodeCount;
    *triangles = m->triangleCount;
}

// The mesh's own tree, 8 floats a node: lower (w: the right child's index, or a
// leaf's first triangle) and upper (w: 0 inner, else the leaf's triangle count),
// indices as int bits, the left child right after its parent; and its triangles
// in leaf order, 12 floats each (three padded corners).
void bb_mesh_export(const void* mesh, float* nodes, float* triangles) {
    const b3MeshData* m = (const b3MeshData*)mesh;
    const b3MeshNode* n = b3GetMeshNodes(m);
    for (int32_t i = 0; i < m->nodeCount; ++i) {
        int32_t leaf = n[i].data.asLeaf.type == 3;
        int32_t link = leaf ? (int32_t)n[i].triangleOffset : i + (int32_t)n[i].data.asNode.childOffset;
        int32_t count = leaf ? (int32_t)n[i].data.asLeaf.triangleCount : 0;
        float* o = nodes + 8 * i;
        o[0] = n[i].lowerBound.x, o[1] = n[i].lowerBound.y, o[2] = n[i].lowerBound.z;
        memcpy(&o[3], &link, sizeof link);
        o[4] = n[i].upperBound.x, o[5] = n[i].upperBound.y, o[6] = n[i].upperBound.z;
        memcpy(&o[7], &count, sizeof count);
    }
    const b3Vec3* v = b3GetMeshVertices(m);
    const b3MeshTriangle* t = b3GetMeshTriangles(m);
    for (int32_t i = 0; i < m->triangleCount; ++i) {
        const int32_t corner[3] = {t[i].index1, t[i].index2, t[i].index3};
        for (int k = 0; k < 3; ++k) {
            float* o = triangles + 12 * i + 4 * k;
            o[0] = v[corner[k]].x, o[1] = v[corner[k]].y, o[2] = v[corner[k]].z, o[3] = 0.0f;
        }
    }
}

uint64_t bb_shape_create_geometry(uint64_t body, int32_t kind, const void* geometry, const float material[3]) {
    b3ShapeDef def = b3DefaultShapeDef();
    def.density = material[0];
    def.baseMaterial.friction = material[1];
    def.baseMaterial.restitution = material[2];
    b3BodyId id = b3LoadBodyId(body);
    b3ShapeId shape = kind == 3 ? b3CreateHullShape(id, &def, (const b3HullData*)geometry)
                                : b3CreateMeshShape(id, &def, (const b3MeshData*)geometry, (b3Vec3){1, 1, 1});
    return b3StoreShapeId(shape);
}
