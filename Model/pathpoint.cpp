#include "pathpoint.h"

PathPoint::PathPoint(const Point &_point, const Action &_action, const int _waitTime): action(_action), point(_point), waitTime(_waitTime)
{
}

