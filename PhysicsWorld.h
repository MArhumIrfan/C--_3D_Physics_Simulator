#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>
#include "Physics.h"
#include "Wind.h"
#include "Terrain.h"
#include "Collision.h"
#include "GameObject.h"

struct WaterVolume {
    float level = -0.5f;
    float buoyancy = 2.0f;         // x body weight when fully submerged
    float damping = 6.0f;          // 1/s, applied as exp(-k*dt): frame-rate independent
    float verticalDamping = 8.0f;  // 1/s
};

struct WorldBounds {
    float halfExtent = 15.0f;
    float restitution = 0.6f;
};

class PhysicsWorld {
public:
    static constexpr float FIXED_DT = 1.0f / 120.0f;
    static constexpr int   SOLVER_ITERATIONS = 4;
    static constexpr float MAX_SPEED = 60.0f;
    static constexpr float GROUND_SKIN = 0.02f;

    std::vector<GameObject> objects;
    Vec3 gravity = {0.0f, -9.81f, 0.0f};

    // Environment lives in the world, not in main.cpp
    WindField wind;
    Terrain terrain;
    WaterVolume water;
    WorldBounds bounds;

    float time = 0.0f;
    int lastContactCount = 0;

    int addObject(GameObject obj) {
        obj.body.prevPosition = obj.body.position;
        objects.push_back(obj);
        return static_cast<int>(objects.size()) - 1;
    }

    void clear() { objects.clear(); time = 0.0f; lastContactCount = 0; }

    GameObject* findPlayer() {
        for (auto& o : objects) if (o.isPlayer) return &o;
        return nullptr;
    }

    // One fixed step: forces -> integrate -> body collisions -> environment.
    void step(float dt) {
        time += dt;
        for (auto& o : objects) {
            o.body.grounded = false;
            integrate(o.body, dt);
        }
        lastContactCount = solveBodyCollisions();
        for (auto& o : objects) {
            RigidBody& b = o.body;
            if (b.inverseMass == 0.0f) continue;
            collideTerrain(b);
            collideBounds(b);
            applyWaterDamping(b, dt);
        }
    }

private:
    float submergedFraction(const RigidBody& b) const {
        float bottom = b.position.y - b.radius;
        return clampf((water.level - bottom) / (2.0f * b.radius), 0.0f, 1.0f);
    }

    void integrate(RigidBody& b, float dt) {
        b.prevPosition = b.position;
        if (b.inverseMass == 0.0f) { b.forceAccumulator = Vec3(); return; }

        float sub = submergedFraction(b);

        // Air drag: 0.5 * rho * Cd * A * |v_rel| * v_rel, only above water
        const float airDensity = 1.225f, Cd = 0.47f;
        float area = 3.14159265f * b.radius * b.radius;
        Vec3 rel = wind.getWindAt(b.position, time) - b.velocity;
        Vec3 drag = rel * (0.5f * airDensity * Cd * area * rel.length() * (1.0f - sub));

        // Buoyancy applied BEFORE integration (was one frame late in main.cpp)
        Vec3 buoyancy = {0.0f, sub * b.mass * -gravity.y * water.buoyancy, 0.0f};

        Vec3 forces = b.forceAccumulator + drag + buoyancy;
        b.acceleration = gravity + forces * b.inverseMass;

        b.velocity += b.acceleration * dt;         // semi-implicit Euler
        float speed = b.velocity.length();
        if (speed > MAX_SPEED) b.velocity *= MAX_SPEED / speed;
        b.position += b.velocity * dt;

        b.forceAccumulator = Vec3();
    }

    // ---- broadphase: uniform spatial hash --------------------------------
    static std::int64_t cellKey(int cx, int cy, int cz) {
        return (static_cast<std::int64_t>(cx) * 73856093LL) ^
               (static_cast<std::int64_t>(cy) * 19349663LL) ^
               (static_cast<std::int64_t>(cz) * 83492791LL);
    }

    void collectPairs(std::vector<std::pair<int, int>>& pairs) const {
        float maxR = 0.5f;
        for (const auto& o : objects) maxR = std::max(maxR, o.body.radius);
        const float cell = 2.0f * maxR;

        std::unordered_map<std::int64_t, std::vector<int>> grid;
        for (int i = 0; i < (int)objects.size(); ++i) {
            const RigidBody& b = objects[i].body;
            int x0 = (int)std::floor((b.position.x - b.radius) / cell);
            int x1 = (int)std::floor((b.position.x + b.radius) / cell);
            int y0 = (int)std::floor((b.position.y - b.radius) / cell);
            int y1 = (int)std::floor((b.position.y + b.radius) / cell);
            int z0 = (int)std::floor((b.position.z - b.radius) / cell);
            int z1 = (int)std::floor((b.position.z + b.radius) / cell);
            for (int x = x0; x <= x1; ++x)
                for (int y = y0; y <= y1; ++y)
                    for (int z = z0; z <= z1; ++z)
                        grid[cellKey(x, y, z)].push_back(i);
        }
        for (const auto& kv : grid) {
            const auto& ids = kv.second;
            for (size_t a = 0; a < ids.size(); ++a)
                for (size_t b = a + 1; b < ids.size(); ++b)
                    pairs.emplace_back(std::min(ids[a], ids[b]), std::max(ids[a], ids[b]));
        }
        std::sort(pairs.begin(), pairs.end());
        pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    }

    int solveBodyCollisions() {
        std::vector<std::pair<int, int>> pairs;
        collectPairs(pairs);

        std::vector<Contact> contacts;
        for (auto& p : pairs) {
            RigidBody& a = objects[p.first].body;
            RigidBody& b = objects[p.second].body;
            if (a.inverseMass == 0.0f && b.inverseMass == 0.0f) continue;
            Contact c;
            if (CollisionSystem::checkSphereSphere(a, b, c)) contacts.push_back(c);
        }
        for (int it = 0; it < SOLVER_ITERATIONS; ++it)
            for (auto& c : contacts) CollisionSystem::resolveVelocity(c);
        for (auto& c : contacts) CollisionSystem::correctPositions(c);
        return (int)contacts.size();
    }

    // ---- environment -----------------------------------------------------
    void collideTerrain(RigidBody& b) {
        Vec3& p = b.position;
        float h = terrain.getHeightAt(p.x, p.z);
        Vec3 n = terrain.getNormalAt(p.x, p.z);

        // Distance to the surface measured along the normal, so a sphere
        // resting on a slope no longer sinks into it.
        float dist = (p.y - h) * n.y;
        if (dist > b.radius + GROUND_SKIN) return;
        if (n.y > 0.5f) b.grounded = true;
        if (dist >= b.radius) return;

        p += n * (b.radius - dist);

        float vn = b.velocity.dot(n);
        if (vn >= 0.0f) return;

        float e = std::min(b.restitution, terrain.restitution);
        if (-vn < CollisionSystem::RESTITUTION_THRESHOLD) e = 0.0f;
        float jn = -(1.0f + e) * vn;                // impulse per unit mass
        b.velocity += n * jn;

        // Coulomb friction on the tangential velocity
        Vec3 vt = b.velocity - n * b.velocity.dot(n);
        float tl = vt.length();
        if (tl > 1e-6f) {
            float mu = std::sqrt(b.friction * terrain.friction);
            float dv = std::min(tl, mu * jn);
            b.velocity -= vt * (dv / tl);
        }
    }

    void collideBounds(RigidBody& b) {
        const float lim = bounds.halfExtent, r = bounds.restitution;
        if (b.position.x < -lim) { b.position.x = -lim; b.velocity.x = std::abs(b.velocity.x) * r; }
        if (b.position.x >  lim) { b.position.x =  lim; b.velocity.x = -std::abs(b.velocity.x) * r; }
        if (b.position.z < -lim) { b.position.z = -lim; b.velocity.z = std::abs(b.velocity.z) * r; }
        if (b.position.z >  lim) { b.position.z =  lim; b.velocity.z = -std::abs(b.velocity.z) * r; }
    }

    // exp(-k*dt): identical behavior at any step size (old code used *=0.9/frame)
    void applyWaterDamping(RigidBody& b, float dt) {
        float sub = submergedFraction(b);
        if (sub <= 0.0f) return;
        float kh = std::exp(-water.damping * sub * dt);
        float kv = std::exp(-water.verticalDamping * sub * dt);
        b.velocity.x *= kh;
        b.velocity.z *= kh;
        b.velocity.y *= kv;
    }
};
