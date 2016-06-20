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
    ~PointsView();

    Points getPoints(void) const { return points; }
    void setPoints(Points _points) { points = _points; }
    std::vector<GroupView*> getGroups(void) const { return groupViews; }

    size_t count(void) const { return groupViews.size(); }

public:
    PointView* getPointViewFromName(const QString);
    PointView* getPointViewFromPoint(const Point& point);

private:
    std::vector<GroupView*> groupViews;
    Points points;
};

#endif // POINTSVIEW_H
