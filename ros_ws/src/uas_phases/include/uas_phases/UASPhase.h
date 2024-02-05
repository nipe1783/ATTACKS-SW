#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "uas/UASState.h"
#include "uas_computer_vision/CVImg.h"

class UASPhase
{   
    public:

        // constructors:
        UASPhase() { }

        // fields:
        std::string phaseName_;

        // methods:
        /**
         * @brief Primary function of all UAS mission phases. Tells the UAS where to go.
         *
         */
        virtual UASState generateDesiredState(CVImg cvImg, UASState uasState) = 0;
        
        /**
         * @brief Determines distance between the UAS and a waypoint.
         */
        double distance(UASState s1, UASState s2);
};