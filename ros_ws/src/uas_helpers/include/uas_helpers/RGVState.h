#pragma once

class RGVState {
    public:
        // constructor
        RGVState();
        RGVState(float ix, float iy, float iz);

        //fields
        // inertial x position
        float ix_;
        // inertial y position
        float iy_;
        // inertial z position
        float iz_;
        
        //methods 

        void updateState(float ix, float iy, float iz);
};