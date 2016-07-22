#include "pathpainter.h"
#include "View/mapview.h"
#include "View/pointsview.h"
#include "View/pointview.h"
#include <QDebug>
#include <QPainterPath>
#include <QPixmap>

PathPainter::PathPainter(MapView * const &mapPixmapItem, PointsView * const &_pointViews) : QGraphicsPathItem(mapPixmapItem), pointViews(_pointViews), mapItem(mapPixmapItem)
{
    setPen(QPen(Qt::red));
}
// save = do i want to save the path changes
void PathPainter::reset(bool save){
    clearPointViews(save);
    path = QPainterPath();
    pathVector.clear();
    setPath(path);
}

void PathPainter::refresh(bool save){
    qDebug() << "pathpainter refresh called";

    clearPointViews();

    if(pathVector.size() > 0){
        PointView* startPointView = NULL;
        PointView* endPointView = NULL;
        for(int i = 0; i < pathVector.size(); i++){
            if(pathVector.at(i).getName().size() > 0){
                QPointF pointCoord = QPointF(pathVector.at(i).getPosition().getX(),
                                                            pathVector.at(i).getPosition().getY());
                PointView* pointView = &(*(pointViews->getPointViewFromPoint(pathVector.at(i))));

                if(pointView != NULL){
                    //qDebug() << "Found permanent pointView";
                } else {
                    //qDebug() << "No permanent pointView";

                    MapView* mapView = (MapView*) parentItem();
                    QVector<PointView*> pointViewVector = mapView->getPathCreationPoints();
                    for(int j = 0; j < pointViewVector.size(); j++){
                        if(pathVector.at(i).comparePos(pointViewVector.at(j)->getPoint()->getPosition().getX(),
                                                       pointViewVector.at(j)->getPoint()->getPosition().getY())){

                            pointView = pointViewVector.at(j);
                            pointView->setAddedToPath(true);
                        }
                    }

                    if(pathVector.at(i).comparePos(mapView->getTmpPointView()->getPoint()->getPosition().getX(),
                                                   mapView->getTmpPointView()->getPoint()->getPosition().getY())){

                        //qDebug() << "But found the unique temporary pointView";
                        pointView = mapView->getTmpPointView();
                        pointView->setAddedToPath(true);
                    }
                }

                if(i == 0){
                    path = QPainterPath(pointCoord);
                    startPointView = pointView;
                } else {
                    path.lineTo(pointCoord);
                }

                if(i == pathVector.size()-1)
                    endPointView = pointView;

               // qDebug() << pointView->getType();
                qDebug() << "avant" ;

                if((pointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                    qDebug() << " oops i changed in refresh";
                    setPointViewPixmap(i, pointView);
                }
            }
        }

        setPath(path);

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                //qDebug() << "changed startend refresh la";
                startPointView->setPixmap(PointView::PixmapType::START_STOP);
            }
        } else {
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                //qDebug() << "changed start refresh la";
                startPointView->setPixmap(PointView::PixmapType::START);
            }
            if((endPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                //qDebug() << "changed end refresh la";
                endPointView->setPixmap(PointView::PixmapType::STOP);
            }
        }
    }
}

void PathPainter::updatePath(const QVector<Point>& pointVector, bool save){
    qDebug() << "path painter updatepath called";
    reset(save);
    pathVector = pointVector;
    refresh(save);

}

void PathPainter::setPointViewPixmap(const int id, PointView* const pointView){
    if(mapItem->getState() == GraphicItemState::EDITING && pointView->getType() == PointView::PixmapType::HOVER){
        qDebug() << "being edited";
        return;
    } else {
        qDebug() << "not editing";
        if(id == 0)
            pointView->setPixmap(PointView::PixmapType::START);

        else if (id == pathVector.size()-1)
            pointView->setPixmap(PointView::PixmapType::STOP);

        else
            pointView->setPixmap(PointView::PixmapType::MID);
    }
}

void PathPainter::clearPointViews(bool save){
    qDebug() << "pathpainter: clear point views";
    pointViews->setNormalPixmaps();

    QVector<PointView*> pointViewVector = mapItem->getPathCreationPoints();
    for(int k = 0; k < pointViewVector.size(); k++){
         if((mapItem->getState() != GraphicItemState::EDITING) || (pointViewVector.at(k)->getType() != PointView::PixmapType::HOVER) || save){
            //qDebug() << "was not orange" << mapItem->getState() << pointViewVector.at(k)->getType() << save;
            pointViewVector.at(k)->setPixmap(PointView::PixmapType::NORMAL);
         }
    }
}

void PathPainter::updatePath(const QVector<PointView*>& pointViewsVector, bool save){
    reset(save);
    if(pointViewsVector.size() > 0){
        PointView* startPointView = NULL;
        PointView* endPointView = NULL;
        for(int i = 0; i < pointViewsVector.size(); i++){
            QPointF pointCoord = QPointF(pointViewsVector.at(i)->getPoint()->getPosition().getX(),
                                                        pointViewsVector.at(i)->getPoint()->getPosition().getY());
            if(i == 0){
                path = QPainterPath(pointCoord);
                startPointView = pointViewsVector.at(i);
            } else {
                path.lineTo(pointCoord);
            }
            if(i == pointViewsVector.size()-1)
                endPointView = pointViewsVector.at(i);
            if((pointViewsVector.at(i)->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                //qDebug() << "was not orange la " << i;
                //qDebug() << pointViewsVector.at(i)->getType() << mapItem->getState();
                setPointViewPixmap(i, pointViewsVector.at(i));
            }
        }
        setPath(path);

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                qDebug() << "changed startend updatepath la";
                startPointView->setPixmap(PointView::PixmapType::START_STOP);
            }
        } else {
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                qDebug() << "changed start updatepath la";
                startPointView->setPixmap(PointView::PixmapType::START);
            }
            if((endPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING) || save){
                qDebug() << "changed start updatepath la";
                endPointView->setPixmap(PointView::PixmapType::STOP);
            }
        }
    }
}
