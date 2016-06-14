#include "pathpainter.h"
#include "View/mapview.h"
#include "View/pointsview.h"
#include "View/pointview.h"
#include <QDebug>
#include <QPainterPath>
#include <QPixmap>

PathPainter::PathPainter(MapView * const &mapPixmapItem, PointsView * const &_pointViews) : QGraphicsPathItem(mapPixmapItem){
    setPen(QPen(Qt::red));
    pointViews = _pointViews;
}

PathPainter::~PathPainter(){
    delete pointViews;
}

void PathPainter::reset(void){
    clearPointViews();
    path = QPainterPath();
    pathVector.clear();
    setPath(path);
}

void PathPainter::refresh(void){
    clearPointViews();
    for(int i = 0; i < pathVector.size(); i++){
        if(pathVector.at(i).getName().size() > 0){
            QPointF pointCoord = QPointF(pathVector.at(i).getPosition().getX(),
                                                        pathVector.at(i).getPosition().getY());
            PointView* pointView = &(*(pointViews->getPointViewFromPoint(pathVector.at(i))));

            if(pointView != NULL){
                setPointViewPixmap(i, pointView);
            } else {

                MapView* mapView = (MapView*) parentItem();
                QVector<PointView*> pointViewVector = mapView->getPathCreationPoints();
                for(int j = 0; j < pointViewVector.size(); j++){
                    if(pathVector.at(i).comparePos(pointViewVector.at(j)->getPoint()->getPosition().getX(),
                                                   pointViewVector.at(j)->getPoint()->getPosition().getY())){
                        pointView = pointViewVector.at(j);
                        pointView->setAddedToPath(true);
                        setPointViewPixmap(i, pointView);
                    }
                }
                if(pathVector.at(i).comparePos(mapView->getTmpPointView()->getPoint()->getPosition().getX(),
                                               mapView->getTmpPointView()->getPoint()->getPosition().getY())){
                    pointView = mapView->getTmpPointView();
                    pointView->setAddedToPath(true);
                    setPointViewPixmap(i, pointView);
                }         }

            if(i == 0){
                path = QPainterPath(pointCoord);
            } else {
                path.lineTo(pointCoord);
            }
        }
    }
    setPath(path);
}

void PathPainter::updatePath(QVector<Point> pointVector){
    reset();
    pathVector = pointVector;
    refresh();
}

void PathPainter::setPointViewPixmap(const int id, PointView * const pointView){
    if(id == 0){
        pointView->setPixmap(QPixmap(PIXMAP_START));
    } else if (id == pathVector.size()-1){
        pointView->setPixmap(QPixmap(PIXMAP_STOP));
    } else {
        pointView->setPixmap(QPixmap(PIXMAP_MID));
    }
}

void PathPainter::clearPointViews(void){
    for(size_t i = 0; i < pointViews->getGroups().size(); i++){
        GroupView groupView = pointViews->getGroups().at(i);
        std::vector<std::shared_ptr<PointView>> pointViews = groupView.getPointViews();
        for(size_t j = 0; j < pointViews.size(); j++){
            pointViews.at(j)->setPixmap(QPixmap(PIXMAP_NORMAL));
        }
    }

    MapView* mapView = (MapView*) parentItem();
    QVector<PointView*> pointViewVector = mapView->getPathCreationPoints();
    for(int k = 0; k < pointViewVector.size(); k++){
        pointViewVector.at(k)->setPixmap(QPixmap(PIXMAP_NORMAL));
    }
}
