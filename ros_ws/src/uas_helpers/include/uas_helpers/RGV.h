#pragma once
#include "RGVState.h"

class RGV {
    public:
        // constructor
        RGV();
        RGV(int id):id_(id){};

        //fields
        int id_;
        RGVState state_;

};