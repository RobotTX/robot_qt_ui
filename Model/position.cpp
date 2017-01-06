#include "position.h"
#include <iostream>
#include <cmath>

Position::Position(): x(0), y(0) {}

Position::Position(const double _x, const double _y): x(_x), y(_y) {}

bool operator==(const Position& pos, const Position& otherPos){
    /// cannot apply == on float numbers
    return std::abs(pos.getX() - otherPos.getX()) < 0.01 && std::abs(pos.getY() - otherPos.getY()) < 0.01;
}

bool operator!=(const Position& pos, const Position& otherPos){
    return ! (pos == otherPos);
}
