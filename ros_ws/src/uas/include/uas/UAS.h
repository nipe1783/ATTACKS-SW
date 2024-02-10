#pragma once
#include "UASState.h"
#include "uas_helpers/Camera.h"
#include <vector>

class UAS {
    public:
        // constructor
        UAS();
        UAS(UASState state);

        //fields
        UASState state_;

        //vector of cameras onboard UAS
        std::vector<Camera> cameras_;

        // methods
        void addCamera(const Camera& camera);

};