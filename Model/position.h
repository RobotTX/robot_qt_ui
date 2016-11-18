#ifndef POSITION_H
#define POSITION_H

/**
 * @brief The Position class
 * Represents a position (x,y)
 */
class Position
{
public:
    Position();
    Position(const double x, const double y);

    double getX(void) const { return x; }
    double getY(void) const { return y; }

    void setX(const double _x) { x = _x; }
    void setY(const double _y) { y = _y; }

private:
    double x;
    double y;
};

/// to easily compare two positions
bool operator==(const Position& pos, const Position& otherPos);
bool operator!=(const Position& pos, const Position& otherPos);

#endif /// POSITION_H
