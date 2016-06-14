#include "groupview.h"
#include "View/pointview.h"

GroupView::GroupView()
{
}

void GroupView::addPointView(const std::shared_ptr<PointView> &pointView){
    pointViews.push_back(pointView);
}
