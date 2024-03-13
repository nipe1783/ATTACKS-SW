#include "uas_phases/UASPhase.h"

class UASExplorationPhase : public UASPhase
{
    public:
        //fields:
        UASExplorationPhase(std::vector<UASState> waypoints);
        std::vector<UASState> waypoints_;
        unsigned int waypointIndex_;
        float distanceThresh_;

        //methods:
        UASState generateDesiredState(const CVImg& cvImg, const UASState& uasState) override;
};