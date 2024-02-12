#pragma once

class UASState {
    public:
        // constructor
        UASState();
        UASState(float ix, float iy, float iz, float ipsi, float bxV, float byV, float bzV);

        //fields
        // inertial x position
        float ix_;
        // inertial y position
        float iy_;
        // inertial z position
        float iz_;
        // inertial psi
        float ipsi_;
        // body x velocity
        float bxV_;
        // body y velocity
        float byV_;
        // body z velocity
        float bzV_;

};