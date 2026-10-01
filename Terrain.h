#pragma once
#include <cmath>
#include "Physics.h"

class Terrain {
public:
    float restitution = 0.5f;
    float friction = 0.6f;

    float getHeightAt(float x, float z) const {
        return std::sin(x * 0.2f) * std::cos(z * 0.2f) * 2.0f;
    }

    // Surface normal via central differences
    Vec3 getNormalAt(float x, float z) const {
        const float d = 0.01f;
        float hL = getHeightAt(x - d, z), hR = getHeightAt(x + d, z);
        float hD = getHeightAt(x, z - d), hU = getHeightAt(x, z + d);
        return Vec3{hL - hR, 2.0f * d, hD - hU}.normalized();
    }
};
