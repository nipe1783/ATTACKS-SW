#include "uas_phases/UASExplorationPhase.h"
#include "uas_computer_vision/CVImg.h"

UASExplorationPhase::UASExplorationPhase(std::vector<UASState> waypoints) : waypoints_(waypoints)
{
    phaseName_ = "Exploration";
    waypointIndex_ = 0;
}

UASState UASExplorationPhase::generateDesiredState(CVImg cvImg, UASState uasState)
{   
    if(distance(uasState, waypoints_[waypointIndex_]) < 0.5) {
        waypointIndex_++;
        if(waypointIndex_ == waypoints_.size()) {
            waypointIndex_ = 0;
        }
    }

    return waypoints_[waypointIndex_];
}

UASState UASExplorationPhase::generateDesiredState(CVImg rgv1, CVImg rgv2, UASState uasState)
{   
    std::cout<<"NOT IMPLEMENTED"<<std::endl;
    return  waypoints_[waypointIndex_];;
}