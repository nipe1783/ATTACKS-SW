#pragma once

class UASState {
    public:
        // constructor
        UASState() : ix(0.0f), iy(0.0f), iz(0.0f), ipsi(0.0f), bxV(0.0f), byV(0.0f), bzV(0.0f) {};
        UASState(float ix, float iy, float iz, float ipsi, float bxV, float byV, float bzV) : ix(ix), iy(iy), iz(iz), ipsi(ipsi), bxV(bxV), byV(byV), bzV(bzV) {};

        //fields
        // inertial x position
        float ix;
        // inertial y position
        float iy;
        // inertial z position
        float iz;
        // inertial psi
        float ipsi;
        // body x velocity
        float bxV;
        // body y velocity
        float byV;
        // body z velocity
        float bzV;
};