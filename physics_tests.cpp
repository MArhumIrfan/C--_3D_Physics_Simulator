// Headless tests: g++ -std=c++17 -Wall -Wextra physics_tests.cpp -o physics_tests
#include <cstdio>
#include "PhysicsWorld.h"
#include "SceneLoader.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("FAIL: %s\n", msg); ++failures; } else std::printf("ok:   %s\n", msg); } while (0)

int main() {
    { // head-on collision conserves momentum
        RigidBody a, b;
        a.position = {0, 0, 0};   a.velocity = { 2, 0, 0}; a.radius = 1;
        b.position = {1.5f, 0, 0}; b.velocity = {-2, 0, 0}; b.radius = 1;
        Contact c;
        bool hit = CollisionSystem::checkSphereSphere(a, b, c);
        CollisionSystem::resolveVelocity(c);
        CHECK(hit, "spheres detect overlap");
        CHECK(std::abs(a.velocity.x + b.velocity.x) < 1e-4f, "momentum conserved");
        CHECK(a.velocity.x < 0 && b.velocity.x > 0, "spheres bounce apart");
    }
    { // ball dropped on a flat terrain peak (7.854, 0) settles on top of it
        PhysicsWorld w;
        w.wind.baseWind = {0, 0, 0};
        w.wind.gustStrength = 0;
        GameObject o;
        o.body.position = {7.854f, 6.0f, 0.0f};
        o.body.setMass(2.0f);
        o.body.radius = 0.5f;
        w.addObject(o);
        for (int i = 0; i < 120 * 6; ++i) w.step(PhysicsWorld::FIXED_DT);
        const RigidBody& b = w.objects[0].body;
        float expected = w.terrain.getHeightAt(b.position.x, b.position.z) + 0.5f;
        std::printf("      rest y=%.3f expected~%.3f speed=%.4f\n", b.position.y, expected, b.velocity.length());
        CHECK(std::abs(b.position.y - expected) < 0.1f, "ball rests on terrain surface");
        CHECK(b.velocity.length() < 0.2f, "ball comes to rest");
        CHECK(b.grounded, "grounded flag set");
    }
    { // pile of bodies stays finite and in bounds
        PhysicsWorld w;
        for (int i = 0; i < 60; ++i) {
            GameObject o;
            o.body.position = {(i % 5) * 0.4f, 5.0f + i * 0.6f, (i % 3) * 0.4f};
            o.body.radius = 0.5f;
            w.addObject(o);
        }
        for (int i = 0; i < 120 * 5; ++i) w.step(PhysicsWorld::FIXED_DT);
        bool sane = true;
        for (auto& o : w.objects)
            if (!std::isfinite(o.body.position.y) || std::abs(o.body.position.x) > 16.0f) sane = false;
        CHECK(sane, "60-body pile stays finite and inside bounds");
    }
    { // scene loader
        PhysicsWorld w;
        std::vector<std::string> errs;
        int n = SceneLoader::loadScene("level.txt", w, &errs);
        CHECK(n == 4 && errs.empty(), "level.txt loads 4 objects, no errors");
        CHECK(w.findPlayer() == &w.objects[0], "player flag parsed");
        CHECK(w.objects[2].shape == Shape::Box, "cube shape parsed");
    }
    std::printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures;
}
