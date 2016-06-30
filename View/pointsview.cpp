#include "pointsview.h"
#include "Model/group.h"
#include "View/groupview.h"
#include "View/pointview.h"
#include "Model/point.h"
#include <QDebug>
#include <QGraphicsItem>

PointsView::PointsView(std::shared_ptr<Points> const& _points, QGraphicsItem* parent): points(_points){
    // for each group
    for(int i = 0; i < points->getGroups().size(); i++){
        //for each point of each group
        GroupView* groupView = new GroupView();
        for(int j = 0; j < points->getGroups().at(i)->getPoints().size(); j++){
            std::shared_ptr<Point> curr_point = points->getGroups().at(i)->getPoints().at(j);
            PointView* pointViewPtr = new PointView(curr_point, parent);
            pointViewPtr->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
            groupView->addPointView(pointViewPtr);
         }
        groupViews.push_back(groupView);
    }
}

PointsView::~PointsView(){
    qDeleteAll(groupViews.begin(), groupViews.end());
    groupViews.clear();
}

PointView* PointsView::getPointViewFromPoint(const Point& newPoint){
    for(size_t i = 0; i < groupViews.size(); i++){
        GroupView* groupView = groupViews.at(i);
        std::vector<PointView*> pointViews = groupView->getPointViews();
        for(size_t j = 0; j < pointViews.size(); j++){
            PointView* pointView = pointViews.at(j);
            std::shared_ptr<Point> point = pointView->getPoint();
            if(*point == newPoint){
                return pointView;
            }
        }
    }
    return NULL;
}

PointView* PointsView::getPointViewFromName(const QString _name){
    for(size_t i = 0; i < groupViews.size(); i++){
        GroupView* groupView = groupViews.at(i);
        std::vector<PointView*> pointViews = groupView->getPointViews();
        for(size_t j = 0; j < pointViews.size(); j++){
            PointView* pointView = pointViews.at(j);
            std::shared_ptr<Point> point = pointView->getPoint();
            if(!point->getName().compare(_name))
                return pointView;
        }
    }
    return 0;
}

void PointsView::addPointView(PointView* const& _pointView){
    std::pair<int, int> pointIndexes = points->findPointIndexes(_pointView->getPoint()->getName());

    GroupView* groupView = groupViews.at(pointIndexes.first);
    qDebug() << "trying to add a pointview to group " << points->getGroups()[pointIndexes.first]->getName() << groupView->getPointViews().size();
    groupView->addPointView(_pointView);
    qDebug() << "trying to add a pointview " << groupView->getPointViews().size();
}

void PointsView::setNormalPixmaps(void){
    for(size_t i = 0; i < groupViews.size(); i++){
        GroupView* groupView = groupViews.at(i);
        for(size_t j = 0; j < groupView->getPointViews().size(); j++){
            groupView->getPointViews().at(j)->setPixmap(PointView::PixmapType::NORMAL);
        }
    }
}
