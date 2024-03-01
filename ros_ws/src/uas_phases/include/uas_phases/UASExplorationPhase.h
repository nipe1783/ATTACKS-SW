#include "uas_phases/UASPhase.h"

class UASExplorationPhase : public UASPhase
{
    public:
        //fields:
        UASExplorationPhase(std::vector<UASState> waypoints);
        std::vector<UASState> waypoints_;
        unsigned int waypointIndex_;

        //methods:
        UASState generateDesiredState(CVImg cvImg, UASState uasState) override;
        UASState generateDesiredState(CVImg rgv1, CVImg rgv2, UASState uasState) override;
};