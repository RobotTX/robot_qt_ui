#include "groupview.h"
#include "View/pointview.h"

GroupView::GroupView(){
}

GroupView::~GroupView(){
    qDeleteAll(pointViews.begin(), pointViews.end());
    pointViews.clear();
}


void GroupView::addPointView(PointView* pointView){
    pointViews.push_back(pointView);
}
