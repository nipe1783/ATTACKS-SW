#pragma once


class Camera {
    public:
        // constructor
        Camera(int id, int width, int height, double xFOV, double yFOV);
        Camera();

        //fields
        int id_;

        //pixel width and height
        int width_;
        int height_;

        // Camera FOV in degrees
        double xFOV_; 
        double yFOV_; 
};