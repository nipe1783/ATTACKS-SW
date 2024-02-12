#pragma once


class Camera {
    public:
        // constructor
        Camera(int width, int height, double FOV);

        //fields
        int id_;

        //pixel width and height
        int width_;
        int height_;

        // Camera FOV
        double FOV_; 
};