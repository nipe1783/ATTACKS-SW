#pragma once

class Waypoint {
    public:
        Waypoint() : x(0), y(0), z(0) {}
        Waypoint(float x, float y, float z) : x(x), y(y), z(z) {}
        float x;
        float y;
        float z;
};