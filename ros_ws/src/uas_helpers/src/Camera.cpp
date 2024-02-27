#include "uas_helpers/Camera.h"

Camera::Camera(int id, int width, int height, double xFOV, double yFOV) : id_(id), width_(width), height_(height), xFOV_(xFOV), yFOV_(yFOV) {}

Camera::Camera() : id_(0), width_(0), height_(0), xFOV_(0), yFOV_(0) {}