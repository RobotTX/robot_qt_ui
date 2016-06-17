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
    Position(float x, float y);

    /// Getters
    float getX(void) const { return x; }
    float getY(void) const { return y; }

    ///Setters
    void setX(const float _x) { x = _x; }
    void setY(const float _y) { y = _y; }

private:
    float x;
    float y;
};

#endif // POSITION_H
