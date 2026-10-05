#pragma once
#include <cmath>

// ---------------------------------------------------------------------------
// Math + core data types. No raylib dependency, so the whole physics layer can
// be compiled and unit-tested headlessly.
// ---------------------------------------------------------------------------
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float lengthSq() const { return dot(*this); }
    float length() const { return std::sqrt(lengthSq()); }
    Vec3 normalized() const {
        float len = length();
        return len > 1e-8f ? *this * (1.0f / len) : Vec3();
    }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a + (b - a) * t; }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// One enum replaces the magic 0/1/2 numbers. Box and Mesh are VISUAL shapes:
// every body currently collides as a sphere of `radius`.
enum class Shape { Sphere = 0, Box = 1, Mesh = 2 };

// Minimal quaternion type used for mesh orientation. The physics layer itself
// does not depend on it for collision detection.
struct Quant {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Contact {
    Vec3 point;       // world-space contact point
    Vec3 normal;      // contact normal
    Vec3 rA;
    Vec3 rB;
    float penetration = 0.0f;
    float impulse = 0.0f;
};

struct RigidBody {
    Vec3 position;
    Vec3 prevPosition;      // previous fixed step, for render interpolation
    Vec3 velocity;
    Vec3 acceleration;
    Vec3 forceAccumulator;
    Vec3 angularVelocity;
    Vec3 torqueAccumulator;

    float invInertia = 1.0f;   // 0 = static / immovable
    float mass = 1.0f;
    float inverseMass = 1.0f;   // 0 = static / immovable
    float radius = 0.5f;
    float restitution = 0.5f;   // bounciness
    float friction = 0.4f;      // Coulomb coefficient
    bool grounded = false;      // touching walkable terrain this step

    Quant orientation;   // for mesh rotation, not used for collision

    void setMass(float m) {
        if (m <= 0.0f) { mass = 0.0f; inverseMass = 0.0f; }
        else           { mass = m;    inverseMass = 1.0f / m; }
    }
};
