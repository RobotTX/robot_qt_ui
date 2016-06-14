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
    std::vector<std::shared_ptr<PointView>> getPointViews(void) const { return pointViews; }
    void addPointView(const std::shared_ptr<PointView> &pointView);

private:
    std::vector<std::shared_ptr<PointView>> pointViews;
};

#endif // GROUPVIEW_H
