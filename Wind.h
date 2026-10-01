#pragma once
#include <cmath>
#include "Physics.h"

class WindField {
public:
    Vec3 baseWind = {5.0f, 0.0f, 2.0f};
    float gustStrength = 2.0f;   // set to 0 for deterministic tests

    Vec3 getWindAt(const Vec3& p, float time) const {
        float gustX = std::sin(time * 2.0f + p.y * 0.1f) * gustStrength;
        float gustZ = std::cos(time * 1.5f + p.x * 0.1f) * gustStrength;
        return {baseWind.x + gustX, baseWind.y, baseWind.z + gustZ};
    }
};
