#pragma once

class UASState {
    public:
        UASState() : x(0), y(0), z(0) {}
        UASState(float x, float y, float z) : x(x), y(y), z(z) {}
        float x;
        float y;
        float z;
};