#include "raylib.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <string>
#include <vector>
#include "PhysicsWorld.h"
#include "SceneLoader.h"

namespace {

constexpr int MAX_OBJECTS = 300;

Vector3 toRl(const Vec3& v) { return Vector3{ v.x, v.y, v.z }; }
Color   toRl(const ColorRGB& c) { return Color{ c.r, c.g, c.b, 255 }; }

// ---- assets: load once, fall back to generated meshes if files are missing --
struct ModelAsset {
    Model model{};
    float nativeRadius = 1.0f;   // so we can scale the mesh to the collision radius
};

ModelAsset loadModelOrGenerate(const char* path, const std::function<Mesh()>& generate) {
    ModelAsset a;
    if (FileExists(path)) a.model = LoadModel(path);
    if (a.model.meshCount == 0) {
        TraceLog(LOG_WARNING, "ASSET: '%s' missing/invalid, using generated mesh", path);
        a.model = LoadModelFromMesh(generate());
    }
    BoundingBox bb = GetModelBoundingBox(a.model);
    float half = std::max({ bb.max.x - bb.min.x, bb.max.y - bb.min.y, bb.max.z - bb.min.z }) * 0.5f;
    if (half > 1e-4f) a.nativeRadius = half;
    return a;
}

// ---- scene ------------------------------------------------------------------
void resetScene(PhysicsWorld& world) {
    world.clear();
    std::vector<std::string> errors;
    int loaded = SceneLoader::loadScene("level.txt", world, &errors);
    for (const auto& e : errors) TraceLog(LOG_WARNING, "SCENE: %s", e.c_str());
    if (loaded == 0) {
        GameObject p;
        p.isPlayer = true;
        p.color = { 255, 0, 0 };
        p.body.position = { 0.0f, 15.0f, 0.0f };
        p.body.radius = 1.0f;
        p.body.setMass(1.0f);
        world.addObject(p);
    }
}

// ---- player: steer with a FORCE toward a target velocity, so wind, collisions
// and momentum still act on the player (old code overwrote velocity.x/z). ----
struct PlayerInput {
    Vec3 moveDir;
    bool jump = false;
};

void applyPlayerControl(PhysicsWorld& world, PlayerInput& in) {
    GameObject* p = world.findPlayer();
    if (!p) return;
    RigidBody& b = p->body;
    const float speed = 8.0f, gain = 12.0f;     // gain: 1/s response rate
    Vec3 target = in.moveDir * speed;
    b.forceAccumulator.x += (target.x - b.velocity.x) * gain * b.mass;
    b.forceAccumulator.z += (target.z - b.velocity.z) * gain * b.mass;
    if (in.jump && b.grounded) {                 // real ground contact, not |vy| < 0.1
        b.velocity.y = 7.0f;
        in.jump = false;
    }
}

void drawTerrain(const Terrain& t, int half) {
    auto pt = [&](int x, int z) { return Vector3{ (float)x, t.getHeightAt((float)x, (float)z), (float)z }; };
    for (int i = -half; i <= half; ++i)
        for (int j = -half; j < half; ++j) {
            DrawLine3D(pt(i, j), pt(i, j + 1), DARKGREEN);
            DrawLine3D(pt(j, i), pt(j + 1, i), DARKGREEN);
        }
}

} // namespace

int main() {
    InitWindow(1280, 720, "3D Physics Engine Sandbox - 2026 Edition");
    SetTargetFPS(60);

    // ---- orbit camera (right mouse drag = look, wheel = zoom) ----
    Camera3D camera = { 0 };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    float camYaw = 0.0f, camPitch = 0.5f, camDist = 17.0f;

    // ---- assets ----
    ModelAsset assets[3] = {
        loadModelOrGenerate("assets/sphere.obj",      [] { return GenMeshSphere(1.0f, 16, 16); }),
        loadModelOrGenerate("assets/cube.obj",        [] { return GenMeshCube(2.0f, 2.0f, 2.0f); }),
        loadModelOrGenerate("assets/custom_prop.obj", [] { return GenMeshTorus(0.5f, 1.0f, 16, 32); }),
    };
    Texture2D texture = LoadTexture("assets/texture.png");
    if (texture.id != 0) {
        for (auto& a : assets)
            if (a.model.materialCount > 0)
                a.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    } else {
        TraceLog(LOG_WARNING, "ASSET: assets/texture.png missing, drawing untextured");
    }
    // Colors come ONLY from GameObject::color (tint). We no longer also set the
    // material color, which used to get multiplied with the tint.

    PhysicsWorld world;
    resetScene(world);

    std::mt19937 rng{ std::random_device{}() };
    std::uniform_real_distribution<float> jitter(-2.0f, 2.0f);

    Shape spawnShape = Shape::Sphere;
    float spawnMass = 1.0f;
    bool paused = false, debugDraw = false;
    float accumulator = 0.0f;
    float physicsMs = 0.0f;
    PlayerInput input;

    while (!WindowShouldClose()) {
        float frameDt = std::min(GetFrameTime(), 0.1f);

        // ---- camera ----
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            camYaw   -= d.x * 0.005f;
            camPitch  = clampf(camPitch + d.y * 0.005f, 0.1f, 1.5f);
        }
        camDist = clampf(camDist - GetMouseWheelMove(), 5.0f, 40.0f);

        // ---- toggles / menu ----
        if (IsKeyPressed(KEY_ONE)) spawnShape = Shape::Sphere;
        if (IsKeyPressed(KEY_TWO)) spawnShape = Shape::Box;
        if (IsKeyPressed(KEY_Z)) spawnMass = 0.2f;
        if (IsKeyPressed(KEY_X)) spawnMass = 1.0f;
        if (IsKeyPressed(KEY_C)) spawnMass = 5.0f;
        if (IsKeyPressed(KEY_P)) paused = !paused;
        if (IsKeyPressed(KEY_G)) debugDraw = !debugDraw;
        if (IsKeyPressed(KEY_R)) resetScene(world);

        if (IsKeyPressed(KEY_M)) {
            if (GameObject* p = world.findPlayer())
                p->shape = static_cast<Shape>((static_cast<int>(p->shape) + 1) % 3);
        }

        if (IsKeyPressed(KEY_SPACE) && (int)world.objects.size() < MAX_OBJECTS) {
            GameObject o;
            float rx = jitter(rng), rz = jitter(rng);
            o.shape = spawnShape;
            o.body.position = { rx, 18.0f, rz };
            o.body.velocity = { rx * 0.5f, 0.0f, rz * 0.5f };
            o.body.setMass(spawnMass);
            o.body.radius = 0.8f * std::cbrt(spawnMass);
            o.color = (spawnMass > 2.0f) ? ColorRGB{ 230, 41, 55 }
                    : (spawnMass < 0.5f) ? ColorRGB{ 0, 228, 48 }
                                         : ColorRGB{ 255, 161, 0 };
            world.addObject(o);
        }

        float& windX = world.wind.baseWind.x;
        if (IsKeyDown(KEY_UP))   windX += 3.0f * frameDt;
        if (IsKeyDown(KEY_DOWN)) windX -= 3.0f * frameDt;
        windX = clampf(windX, -15.0f, 15.0f);

        // ---- player input, relative to camera heading ----
        Vec3 fwd   = { -std::sin(camYaw), 0.0f, -std::cos(camYaw) };
        Vec3 right = {  std::cos(camYaw), 0.0f, -std::sin(camYaw) };
        Vec3 dir;
        if (IsKeyDown(KEY_W)) dir += fwd;
        if (IsKeyDown(KEY_S)) dir -= fwd;
        if (IsKeyDown(KEY_D)) dir += right;
        if (IsKeyDown(KEY_A)) dir -= right;
        input.moveDir = dir.normalized();
        if (IsKeyPressed(KEY_E)) input.jump = true;

        // ---- fixed-timestep simulation ----
        double t0 = GetTime();
        if (!paused) {
            accumulator += frameDt;
            while (accumulator >= PhysicsWorld::FIXED_DT) {
                applyPlayerControl(world, input);
                world.step(PhysicsWorld::FIXED_DT);
                accumulator -= PhysicsWorld::FIXED_DT;
            }
        }
        input.jump = false;   // don't let a stale press linger
        physicsMs = (float)((GetTime() - t0) * 1000.0);
        float alpha = paused ? 1.0f : accumulator / PhysicsWorld::FIXED_DT;

        auto renderPos = [&](const GameObject& o) { return lerp(o.body.prevPosition, o.body.position, alpha); };

        // ---- camera follows player's interpolated position ----
        Vec3 focus = { 0.0f, 0.0f, 0.0f };
        if (GameObject* p = world.findPlayer()) focus = renderPos(*p);
        camera.target = toRl(focus);
        camera.position = toRl(focus + Vec3{ std::sin(camYaw) * std::cos(camPitch),
                                             std::sin(camPitch),
                                             std::cos(camYaw) * std::cos(camPitch) } * camDist);

        // ---- draw ----
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

            drawTerrain(world.terrain, (int)world.bounds.halfExtent);
            float ext = world.bounds.halfExtent * 2.0f;
            DrawCube(Vector3{ 0.0f, world.water.level, 0.0f }, ext, 0.1f, ext, Fade(BLUE, 0.4f));

            for (const auto& o : world.objects) {
                Vec3 p = renderPos(o);
                const ModelAsset& a = assets[static_cast<int>(o.shape)];
                float scale = o.body.radius / a.nativeRadius;   // visual size == collision size
                DrawModel(a.model, toRl(p), scale, toRl(o.color));
                if (debugDraw)
                    DrawLine3D(toRl(p), toRl(p + o.body.velocity * 0.3f), RED);
            }

        EndMode3D();

        // ---- HUD ----
        const GameObject* player = world.findPlayer();
        DrawRectangle(10, 10, 400, 230, Fade(SKYBLUE, 0.85f));
        DrawRectangleLines(10, 10, 400, 230, BLUE);
        DrawText("2026 ENGINE SANDBOX - CONTROL PANEL", 20, 20, 10, DARKGRAY);
        DrawText(TextFormat("Objects: %d / %d   Contacts: %d", (int)world.objects.size(), MAX_OBJECTS, world.lastContactCount), 20, 40, 10, BLACK);
        DrawText(TextFormat("Wind X: %.1f m/s   Physics: %.2f ms", windX, physicsMs), 20, 60, 10, BLACK);
        DrawText(TextFormat("Player shape (M): %d   Grounded: %s", player ? (int)player->shape : -1,
                            (player && player->body.grounded) ? "yes" : "no"), 20, 80, 10, DARKGRAY);
        DrawText(TextFormat("Spawn: shape %d (1/2), mass %.1f (Z/X/C)", (int)spawnShape, spawnMass), 20, 100, 10, DARKGRAY);
        DrawText("[SPACE] Spawn | [WASD] Move | [E] Jump", 20, 130, 10, DARKGRAY);
        DrawText("[M] Swap player model | [R] Reload map", 20, 150, 10, DARKGRAY);
        DrawText("[P] Pause | [G] Velocity debug | [UP/DOWN] Wind", 20, 170, 10, DARKGRAY);
        DrawText("[RIGHT MOUSE drag] Orbit | [WHEEL] Zoom", 20, 190, 10, DARKGRAY);
        if (paused) DrawText("PAUSED", 20, 210, 20, RED);
        DrawFPS(1180, 10);

        EndDrawing();
    }

    UnloadTexture(texture);
    for (auto& a : assets) UnloadModel(a.model);
    CloseWindow();
    return 0;
}
