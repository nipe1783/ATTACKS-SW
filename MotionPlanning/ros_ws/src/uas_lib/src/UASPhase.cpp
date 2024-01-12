#include "uas_lib/UASPhase.h"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "uas_lib/UASState.h"
#include "computer_vision/Blob.h"

double UASPhase::distance(UASState s1, UASState s2)
{
    return sqrt(pow(s1.x - s2.x, 2) + pow(s1.y - s2.y, 2) + pow(s1.z - s2.z, 2));
}