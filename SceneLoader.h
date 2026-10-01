#pragma once
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "GameObject.h"
#include "PhysicsWorld.h"

// Line format (whitespace separated, '#' starts a comment line):
//   shape x y z mass radius r g b [isPlayer] [restitution]
//   shape: 0=Sphere 1=Box 2=Mesh
class SceneLoader {
public:
    // Returns the number of objects loaded (0 if file missing or empty).
    // Problems are reported with line numbers via `errors` instead of being skipped silently.
    static int loadScene(const std::string& filename, PhysicsWorld& world,
                         std::vector<std::string>* errors = nullptr) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            report(errors, "cannot open '" + filename + "'");
            return 0;
        }

        int loaded = 0, lineNo = 0;
        bool havePlayer = false;
        int firstIndex = -1;
        std::string line;

        while (std::getline(file, line)) {
            ++lineNo;
            size_t first = line.find_first_not_of(" \t\r");
            if (first == std::string::npos || line[first] == '#') continue;

            std::istringstream ss(line);
            int shape, r, g, b;
            float x, y, z, mass, radius;
            if (!(ss >> shape >> x >> y >> z >> mass >> radius >> r >> g >> b)) {
                report(errors, "line " + std::to_string(lineNo) + ": expected 9 numeric fields");
                continue;
            }
            if (shape < 0 || shape > 2) {
                report(errors, "line " + std::to_string(lineNo) + ": shape must be 0, 1 or 2");
                continue;
            }
            if (radius <= 0.0f || mass < 0.0f) {
                report(errors, "line " + std::to_string(lineNo) + ": radius must be > 0 and mass >= 0");
                continue;
            }

            int isPlayer = 0;
            float restitution = -1.0f;
            ss >> isPlayer;
            ss >> restitution;

            GameObject obj;
            obj.shape = static_cast<Shape>(shape);
            obj.color = { clampByte(r), clampByte(g), clampByte(b) };
            obj.isPlayer = (isPlayer != 0) && !havePlayer;
            obj.body.position = {x, y, z};
            obj.body.setMass(mass);
            obj.body.radius = radius;
            if (restitution >= 0.0f) obj.body.restitution = std::min(restitution, 1.0f);

            havePlayer = havePlayer || obj.isPlayer;
            int idx = world.addObject(obj);
            if (firstIndex < 0) firstIndex = idx;
            ++loaded;
        }

        // Backwards compatible: files without a player flag use the first object
        if (loaded > 0 && !havePlayer) world.objects[firstIndex].isPlayer = true;
        return loaded;
    }

private:
    static std::uint8_t clampByte(int v) { return (std::uint8_t)std::max(0, std::min(255, v)); }
    static void report(std::vector<std::string>* e, const std::string& msg) { if (e) e->push_back(msg); }
};
