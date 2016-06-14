#ifndef POINTSVIEW_H
#define POINTSVIEW_H

class Group;
class GroupView;
class PointView;

#include "View/groupview.h"
#include "Model/points.h"

/**
 * @brief The PointsView class
 * This class provides a graphical object that represents a Points object, it is represented by a vector of GroupView objects
 */
class PointsView {
public:
    PointsView(const Points& _points);

    Points getPoints(void) const { return points; }
    std::vector<GroupView> getGroups(void) const { return groupViews; }
    std::shared_ptr<PointView> getPointViewFromPoint(const Point& point);

private:
    std::vector<GroupView> groupViews;
    Points points;
};

#endif // POINTSVIEW_H
