#include "uas_helpers/RGV.h"
#include <chrono>

RGV::RGV(int id, int hLow, int hHigh, int sLow, int sHigh, int vLow, int vHigh){
    id_ = id;
    hLow_ = hLow;
    hHigh_ = hHigh;
    sLow_ = sLow;
    sHigh_ = sHigh;
    vLow_ = vLow;
    vHigh_ = vHigh;
    currentPhase_ = "exploration";
    phaseStartTime_ = std::chrono::system_clock::now();

}


RGV::RGV(){
    id_ = 0;
    hLow_ = 255;
    hHigh_ = 255;
    sLow_ = 255;
    sHigh_ = 255;
    vLow_ = 255;
    vHigh_ = 255;
    currentPhase_ = "exploration";
    phaseStartTime_ = std::chrono::system_clock::now();
}
