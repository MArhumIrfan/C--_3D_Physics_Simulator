#pragma once
#include <cstdint>
#include "Physics.h"

struct ColorRGB {
    std::uint8_t r = 255, g = 255, b = 255;
};

// Physics owns position (body.position). Rendering reads it (optionally
// interpolated) instead of keeping a second copy that can drift.
struct GameObject {
    RigidBody body;
    Shape shape = Shape::Sphere;   // visual model AND what M hot-swaps
    ColorRGB color;
    bool isPlayer = false;
};
