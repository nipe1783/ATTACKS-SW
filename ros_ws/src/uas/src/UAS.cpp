#include "uas/UAS.h"
#include "uas/UASState.h"
#include "uas_helpers/Camera.h"

#include <vector>

       // Default constructor
UAS::UAS() {
    //Initialize UAS state with UASState constructor
    state_ = UASState();
    // Initialize cameras as an empty vector
    cameras_ = std::vector<Camera>();
}

// Constructor with UASState parameter
UAS::UAS(UASState state) : state_(state) {
    // Initialize cameras as an empty vector
    cameras_ = std::vector<Camera>();
}

// Method to add a camera
void UAS::addCamera(const Camera& camera) {
    cameras_.push_back(camera);
}
