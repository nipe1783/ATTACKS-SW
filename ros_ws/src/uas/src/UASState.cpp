#include "uas/UASState.h"
#include <cmath>

UASState::UASState() : ix_(0.0f), iy_(0.0f), iz_(0.0f), iphi_(0.0f), itheta_(0.0f), ipsi_(0.0f), bxV_(0.0f), byV_(0.0f), bzV_(0.0f) {};
UASState::UASState(float ix, float iy, float iz, float ipsi, float bxV, float byV, float bzV) : ix_(ix), iy_(iy), iz_(iz), iphi_(0.0f), itheta_(0.0f), ipsi_(ipsi), bxV_(bxV), byV_(byV), bzV_(bzV) {};


void UASState::updateState(float ix, float iy, float iz, float ipsi, float bxV, float byV, float bzV){
    ix_ = ix;
    iy_ = iy;
    iz_ = iz;
    ipsi_ = ipsi;
    bxV_ = bxV;
    byV_ = byV;
    bzV_ = bzV;
};
void UASState::updateAttitude(float q0, float q1, float q2, float q3){
    itheta_ = std::asin(-2.0 * (q0 * q2 - q3 * q1));
    iphi_ = std::atan2(2.0 * (q0 * q1 + q3 * q2), q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2);
};
