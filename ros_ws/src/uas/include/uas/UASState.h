#pragma once

class UASState {
    public:
        // constructor
        UASState();
        UASState(float ix, float iy, float iz, float ipsi, float bxV, float byV, float bzV);

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