#pragma once
#include "RGVState.h"
#include <string>
#include <chrono>

class RGV {
    public:
        // constructor
        RGV(int id, int hLow, int hHigh, int sLow, int sHigh, int vLow, int vHigh);
        RGV();
        ~RGV() = default;

        //fields
        int id_;
        RGVState state_;
        int hLow_;
        int hHigh_;
        int sLow_;
        int sHigh_;
        int vLow_;
        int vHigh_;
        std::string currentPhase_;
        std::chrono::time_point<std::chrono::system_clock> phaseStartTime_;
};      