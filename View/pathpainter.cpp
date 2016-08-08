#include "pathpainter.h"
#include "View/mapview.h"
#include "Model/points.h"
#include "Model/point.h"
#include <QDebug>
#include <QPainterPath>
#include <QPixmap>
#include "Controller/mainwindow.h"
#include "Model/pathpoint.h"

PathPainter::PathPainter(MainWindow* const &mainWindow, MapView* const &mapPixmapItem, std::shared_ptr<Points> _points)
    : QGraphicsPathItem(mapPixmapItem), points(_points), mainWindow(mainWindow), mapView(mapPixmapItem){
    setPen(QPen(Qt::red));
}

void PathPainter::resetPathSlot(void){
    qDebug() << "PathPainter::resetPathSlot called";
    path = QPainterPath();
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    if(points->getGroups()->value(PATH_GROUP_NAME))
        points->getGroups()->value(PATH_GROUP_NAME)->clear();
    currentPath.clear();
    setPath(path);
}

void PathPainter::displayPath(void){
    qDebug() << "\nPathPainter::displayPath called";
    for(int i = 0; i < currentPath.size(); i++){
        qDebug() << i << ":" << currentPath.at(i)->getPoint().getName()
                 << currentPath.at(i)->getPoint().getPosition().getX()
                 << currentPath.at(i)->getPoint().getPosition().getY()
                 << (int) currentPath.at(i)->getAction()
                 << currentPath.at(i)->getWaitTime();
    }
    qDebug() << "\n";
}

void PathPainter::addPathPointSlot(QString name, double x, double y){
    qDebug() << "PathPainter::addPathPointSlot called" << name << x << y;

    int nb = nbUsedPointView(name, x, y);
    std::shared_ptr<PointView> pointView = points->findPathPointView(x, y);
    /// If found, it's a permanent point which is already in the path
    if(nb > 0 && pointView){
        points->addPoint(PATH_GROUP_NAME, pointView);
    } else {
        name = PATH_POINT_NAME + QString::number(currentPath.size()+1) ;
        points->addPoint(PATH_GROUP_NAME, name, x, y, true, Point::PointType::PATH, mapView, mainWindow);
    }

    points->getGroups()->value(PATH_GROUP_NAME)->last()->setState(GraphicItemState::CREATING_PATH);
    Point point = *(points->getGroups()->value(PATH_GROUP_NAME)->last()->getPoint());
    currentPath.push_back(std::shared_ptr<PathPoint>(new PathPoint(point, PathPoint::Action::WAIT)));

    /// Update the path painter
    updatePathPainterSlot();
}

void PathPainter::orderPathPointChangedSlot(int from, int to){
    qDebug() << "PathPainter::updatePathPainter called from" << from << "to" << to;

    /// Do the change in the model
    std::shared_ptr<PathPoint> pathPoint = currentPath.takeAt(from);
    std::shared_ptr<PointView> pointView = points->getGroups()->value(PATH_GROUP_NAME)->takeAt(from);

    if(to > currentPath.size()){
        currentPath.push_back(pathPoint);
        points->getGroups()->value(PATH_GROUP_NAME)->push_back(pointView);
    } else {
        if(from < to){
            currentPath.insert(to-1, pathPoint);
            points->getGroups()->value(PATH_GROUP_NAME)->insert(to-1, pointView);
        } else {
            currentPath.insert(to, pathPoint);
            points->getGroups()->value(PATH_GROUP_NAME)->insert(to, pointView);
        }
    }

    /// Update the path painter
    updatePathPainterSlot();
}

void PathPainter::deletePathPointSlot(int id){
    qDebug() << "PathPainter::deletePathPointSlot called";

    /// Remove the item from the model
    currentPath.remove(id);
    points->getGroups()->value(PATH_GROUP_NAME)->remove(id);

    /// Update the path painter
    updatePathPainterSlot();
}

void PathPainter::editPathPointSlot(int id, QString name, double x, double y){
    qDebug() << "PathPainter::editPathPointSlot called" << id << name << x << y;

    points->getGroups()->value(PATH_GROUP_NAME)->remove(id);
    std::shared_ptr<PointView> newPointView = points->findPointView(name);

    /// If found, it's a permanent point else it's a temporary point
    if(newPointView){
        qDebug() << "PathPainter::editPathPointSlot editing a permanent point";
        points->insertPoint(PATH_GROUP_NAME, id, newPointView);
        currentPath.at(id)->setPoint(*(newPointView->getPoint()));
    } else {
        qDebug() << "PathPainter::editPathPointSlot editing a tmpPoint";
    }

    /// Update the path painter
    updatePathPainterSlot();
}

void PathPainter::actionChangedSlot(int id, QString waitTimeStr){
    qDebug() << "PathPainter::actionChangedSlot called" << id << waitTimeStr;
    PathPoint::Action action;
    int waitTime = 0;
    if(waitTimeStr.compare("") == 0){
        action = PathPoint::Action::HUMAN_ACTION;
    } else {
        action = PathPoint::Action::WAIT;
        waitTime = waitTimeStr.toInt();
    }
    currentPath.at(id)->setAction(action);
    currentPath.at(id)->setWaitTime(waitTime);
}

void PathPainter::updateCurrentPath(void){
    for(int i = 0; i < currentPath.size(); i++){
        currentPath.at(i)->setPoint(*(points->getGroups()->value(PATH_GROUP_NAME)->at(i)->getPoint()));
    }
    displayPath();
}

void PathPainter::updatePathPainterSlot(void){
    qDebug() << "PathPainter::updatePathPainter called";
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    std::shared_ptr<QVector<std::shared_ptr<PointView>>> group = points->getGroups()->value(PATH_GROUP_NAME);

    std::shared_ptr<PointView> startPointView = std::shared_ptr<PointView>();
    std::shared_ptr<PointView> endPointView = std::shared_ptr<PointView>();
    std::shared_ptr<PointView> currentPointView = std::shared_ptr<PointView>();

    if(group && group->size() > 0){
        for(int i = 0; i < group->size(); i++){
            currentPointView = group->at(i);
            currentPointView->setPixmap(PointView::PixmapType::MID);
            QPointF pointCoord = QPointF(currentPointView->getPoint()->getPosition().getX(),
                                         currentPointView->getPoint()->getPosition().getY());

            if(i == 0){
                path = QPainterPath(pointCoord);
                startPointView = currentPointView;
            } else {
                path.lineTo(pointCoord);
            }

            if(i == group->size()-1)
                endPointView = currentPointView;
        }

        setPath(path);

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            startPointView->setPixmap(PointView::PixmapType::START_STOP);
        } else {
            startPointView->setPixmap(PointView::PixmapType::START);
            endPointView->setPixmap(PointView::PixmapType::STOP);
        }
    } else {
        resetPathSlot();
    }
    displayPath();
}

void PathPainter::updatePathPainterPointViewSlot(void){
    qDebug() << "PathPainter::updatePathPainterPointViewSlot called";
    std::shared_ptr<QVector<std::shared_ptr<PointView>>> group = points->getGroups()->value(PATH_GROUP_NAME);

    std::shared_ptr<PointView> startPointView = std::shared_ptr<PointView>();
    std::shared_ptr<PointView> endPointView = std::shared_ptr<PointView>();
    std::shared_ptr<PointView> currentPointView = std::shared_ptr<PointView>();

    if(group && group->size() > 0){
        for(int i = 0; i < group->size(); i++){
            currentPointView = group->at(i);

            if(i == 0){
                startPointView = currentPointView;
            } else {
                currentPointView->setPixmap(PointView::PixmapType::MID);
            }

            if(i == group->size()-1)
                endPointView = currentPointView;
        }

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            startPointView->setPixmap(PointView::PixmapType::START_STOP);
        } else {
            startPointView->setPixmap(PointView::PixmapType::START);
            endPointView->setPixmap(PointView::PixmapType::STOP);
        }
    }
}

int PathPainter::nbUsedPointView(QString name, double x, double y){
    qDebug() << "PathPainter::nbUsedPointView called";
    int nbUsed = 0;
    if(name.contains(PATH_POINT_NAME)){
        for(int i = 0; i < currentPath.size(); i++){
            if(currentPath.at(i)->getPoint().comparePos(x, y))
                nbUsed++;
        }
    } else {
        for(int i = 0; i < currentPath.size(); i++){
            if(currentPath.at(i)->getPoint().getName().compare(name) == 0)
                nbUsed++;
        }
    }
    return nbUsed;
}

void PathPainter::setCurrentPath(const QVector<std::shared_ptr<PathPoint>>& _currentPath){
    for(int i = 0; i < _currentPath.size(); i++){
        Point point = _currentPath.at(i)->getPoint();
        addPathPointSlot(point.getName(), point.getPosition().getX(), point.getPosition().getY());
    }
}


/*
// save = do i want to save the path changes
void PathPainter::reset(bool save){
    qDebug() << "PathPainter reset called";
    clearPointViews(save);
    qDebug() << "pointviews cleared in pathpainter::reset";
    path = QPainterPath();
    currentPath.clear();
    qDebug() << "currentPath also cleared";
    setPath(path);
}

void PathPainter::refresh(bool save){
    qDebug() << "pathpainter refresh called";

    clearPointViews(save);

    if(currentPath.size() > 0){
        PointView* startPointView = NULL;
        PointView* endPointView = NULL;
        for(int i = 0; i < currentPath.size(); i++){
            if(currentPath.at(i).getName().size() > 0){
                QPointF pointCoord = QPointF(currentPath.at(i).getPosition().getX(),
                                                            currentPath.at(i).getPosition().getY());
                PointView* pointView = &(*(pointViews->getPointViewFromPoint(currentPath.at(i))));

                if(pointView != NULL){
                    //qDebug() << "Found permanent pointView";
                } else {
                    //qDebug() << "No permanent pointView";

                    MapView* mapView = (MapView*) parentItem();
                    QVector<PointView*> pointViewVector = mapView->getPathCreationPoints();
                    for(int j = 0; j < pointViewVector.size(); j++){
                        if(currentPath.at(i).comparePos(pointViewVector.at(j)->getPoint()->getPosition().getX(),
                                                       pointViewVector.at(j)->getPoint()->getPosition().getY())){

                            pointView = pointViewVector.at(j);
                            pointView->setAddedToPath(true);
                        }
                    }

                    if(currentPath.at(i).comparePos(mapView->getTmpPointView()->getPoint()->getPosition().getX(),
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

                if(i == currentPath.size()-1)
                    endPointView = pointView;

               // qDebug() << pointView->getType();
                //qDebug() << "avant" ;

                if((pointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                    //qDebug() << " oops i changed in refresh";
                    setPointViewPixmap(i, pointView);
                }
            }
        }

        setPath(path);

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                //qDebug() << "changed startend refresh la";
                startPointView->setPixmap(PointView::PixmapType::START_STOP);
            }
        } else {
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                //qDebug() << "changed start refresh la";
                startPointView->setPixmap(PointView::PixmapType::START);
            }
            if((endPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                //qDebug() << "changed end refresh la";
                endPointView->setPixmap(PointView::PixmapType::STOP);
            }
        }
    }
}

void PathPainter::updatePath(const QVector<Point>& pointVector, bool save){
    qDebug() << "path painter updatepath called";
    reset(save);
    currentPath = pointVector;
    refresh(save);
}

void PathPainter::setPointViewPixmap(const int id, PointView* const pointView){
    qDebug() << "path painter setPointViewPixmap called";
    if(mapItem->getState() == GraphicItemState::EDITING_PATH && pointView->getType() == PointView::PixmapType::HOVER){
        qDebug() << "being edited";
        return;
    } else {
        qDebug() << "not editing" << pointView->getPoint()->getName();
        if(currentPath.last().comparePos(pointView->getPoint()->getPosition()) && currentPath.front().comparePos(pointView->getPoint()->getPosition())){
            qDebug() << "setting start stop pixmap";
            pointView->setPixmap(PointView::PixmapType::START_STOP);
        }
        else if(id == 0)
            pointView->setPixmap(PointView::PixmapType::START);
        else if (id == currentPath.size()-1)
            pointView->setPixmap(PointView::PixmapType::STOP);
        else
            pointView->setPixmap(PointView::PixmapType::MID);
    }
}

void PathPainter::clearPointViews(bool save){
    qDebug() << "pathpainter: clear point views";
    //pointViews->setNormalPixmaps();
    qDebug() << "normal pixmaps set";
    QVector<PointView*> pointViewVector = mapItem->getPathCreationPoints();
    for(int k = 0; k < pointViewVector.size(); k++){
         if((mapItem->getState() != GraphicItemState::EDITING_PATH) || (pointViewVector.at(k)->getType() != PointView::PixmapType::HOVER) || save){
            //qDebug() << "was not orange" << mapItem->getState() << pointViewVector.at(k)->getType() << save;
            pointViewVector.at(k)->setPixmap(PointView::PixmapType::NORMAL);
         }
    }
}

void PathPainter::updatePath(const QVector<PointView*>& pointViewsVector, bool save){
    qDebug() << "path painter updatePath called";
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
            if((pointViewsVector.at(i)->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                //qDebug() << "was not orange la " << i;
                //qDebug() << pointViewsVector.at(i)->getType() << mapItem->getState();
                setPointViewPixmap(i, pointViewsVector.at(i));
            }
        }
        setPath(path);

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                qDebug() << "changed startend updatepath la";
                startPointView->setPixmap(PointView::PixmapType::START_STOP);
            }
        } else {
            if((startPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                qDebug() << "changed start updatepath la";
                startPointView->setPixmap(PointView::PixmapType::START);
            }
            if((endPointView->getType() != PointView::PixmapType::HOVER) || (mapItem->getState() != GraphicItemState::EDITING_PATH) || save){
                qDebug() << "changed start updatepath la";
                endPointView->setPixmap(PointView::PixmapType::STOP);
            }
        }
    }
}
*/
