#pragma once
#include <algorithm>
#include <cmath>
#include "Physics.h"

struct Contact {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    Vec3 normal;              // unit vector pointing from A to B
    float penetration = 0.0f;
};

class CollisionSystem {
public:
    // Impacts slower than this don't bounce, which stops resting micro-jitter.
    static constexpr float RESTITUTION_THRESHOLD = 1.0f;
    static constexpr float PENETRATION_SLOP = 0.01f;
    static constexpr float CORRECTION_PERCENT = 0.4f;

    static bool checkSphereSphere(RigidBody& a, RigidBody& b, Contact& c) {
        Vec3 d = b.position - a.position;
        float distSq = d.lengthSq();
        float radiiSum = a.radius + b.radius;
        if (distSq >= radiiSum * radiiSum) return false;

        float dist = std::sqrt(distSq);
        c.bodyA = &a;
        c.bodyB = &b;
        if (dist < 1e-6f) {                 // perfectly overlapping
            c.normal = {0.0f, 1.0f, 0.0f};
            c.penetration = radiiSum;
        } else {
            c.normal = d * (1.0f / dist);
            c.penetration = radiiSum - dist;
        }
        return true;
    }

    // Normal impulse (bounce) + Coulomb friction impulse. Call several times
    // per step over all contacts for stability.
    static void resolveVelocity(Contact& c) {
        RigidBody& a = *c.bodyA;
        RigidBody& b = *c.bodyB;
        float invSum = a.inverseMass + b.inverseMass;
        if (invSum <= 0.0f) return;

        Vec3 rv = b.velocity - a.velocity;
        float vn = rv.dot(c.normal);
        if (vn > 0.0f) return;              // already separating

        float e = std::min(a.restitution, b.restitution);
        if (-vn < RESTITUTION_THRESHOLD) e = 0.0f;

        float jn = -(1.0f + e) * vn / invSum;
        Vec3 impulse = c.normal * jn;
        a.velocity -= impulse * a.inverseMass;
        b.velocity += impulse * b.inverseMass;

        // Friction along the tangent, clamped by mu * normal impulse
        rv = b.velocity - a.velocity;
        Vec3 tangent = rv - c.normal * rv.dot(c.normal);
        if (tangent.lengthSq() > 1e-10f) {
            tangent = tangent.normalized();
            float mu = std::sqrt(a.friction * b.friction);
            float jt = clampf(-rv.dot(tangent) / invSum, -jn * mu, jn * mu);
            Vec3 fi = tangent * jt;
            a.velocity -= fi * a.inverseMass;
            b.velocity += fi * b.inverseMass;
        }
    }

    // Push overlapping bodies apart, softly (slop + percent) to avoid jitter.
    static void correctPositions(const Contact& c) {
        RigidBody& a = *c.bodyA;
        RigidBody& b = *c.bodyB;
        float invSum = a.inverseMass + b.inverseMass;
        if (invSum <= 0.0f) return;
        float mag = std::max(c.penetration - PENETRATION_SLOP, 0.0f) / invSum * CORRECTION_PERCENT;
        Vec3 corr = c.normal * mag;
        a.position -= corr * a.inverseMass;
        b.position += corr * b.inverseMass;
    }
};
