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
    Position(double x, double y);

    /// Getters
    double getX(void) const { return x; }
    double getY(void) const { return y; }

    ///Setters
    void setX(const double _x) { x = _x; }
    void setY(const double _y) { y = _y; }

private:
    double x;
    double y;
};

#endif // POSITION_H
