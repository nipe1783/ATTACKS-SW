#include "uas/UASState.h"

UASState::UASState() : ix_(0.0f), iy_(0.0f), iz_(0.0f), ipsi_(0.0f), bxV_(0.0f), byV_(0.0f), bzV_(0.0f) {};
UASState::UASState(float ix, float iy, float iz, float ipsi, float bxV, float byV, float bzV) : ix_(ix), iy_(iy), iz_(iz), ipsi_(ipsi), bxV_(bxV), byV_(byV), bzV_(bzV) {};
