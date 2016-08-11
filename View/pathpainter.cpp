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
        if(!points->isAPoint(name, x, y))
            name = PATH_POINT_NAME + QString::number(currentPath.size()+1);
        points->addPoint(PATH_GROUP_NAME, name, x, y, true, Point::PointType::PATH, mapView, mainWindow);
    }

    points->getGroups()->value(PATH_GROUP_NAME)->last()->setState(mapView->getState());
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

void PathPainter::setCurrentPath(QVector<std::shared_ptr<PathPoint>> _currentPath){
    for(int i = 0; i < _currentPath.size(); i++){
        Point point = _currentPath.at(i)->getPoint();
        addPathPointSlot(point.getName(), point.getPosition().getX(), point.getPosition().getY());
    }
}
