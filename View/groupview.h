#ifndef GROUPVIEW_H
#define GROUPVIEW_H

class PointView;


#include <memory>
#include <vector>
/**
 * @brief The GroupView class
 * The purpose of this class is to create a View object that contains the points to display on the map
 */
class GroupView
{
public:
    GroupView();
    ~GroupView();
    std::vector<PointView*> getPointViews(void) const { return pointViews; }
    void addPointView(PointView* pointView);

private:
    std::vector<PointView*> pointViews;
};

#endif // GROUPVIEW_H
