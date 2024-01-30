#include "uas_phases/UASPhase.h"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "uas_lib/UASState.h"
#include "computer_vision/Blob.h"

double UASPhase::distance(UASState s1, UASState s2)
{
    return sqrt(pow(s1.ix - s2.ix, 2) + pow(s1.iy - s2.iy, 2) + pow(s1.iz - s2.iz, 2));
}