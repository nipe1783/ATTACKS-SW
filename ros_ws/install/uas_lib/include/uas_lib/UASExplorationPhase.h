#include "uas_lib/UASPhase.h"

class UASExplorationPhase : public UASPhase
{
    public:
        //fields:
        UASExplorationPhase(std::vector<UASState> waypoints);
        std::vector<UASState> waypoints_;
        unsigned int waypointIndex_;

        //methods:
        UASState generateDesiredState(CVImg cvImg, UASState uasState) override;
};