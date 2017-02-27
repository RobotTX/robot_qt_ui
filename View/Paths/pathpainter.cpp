#include "pathpainter.h"
#include <QDebug>
#include <QPainterPath>
#include <QPixmap>
#include "Controller/mainwindow.h"
#include "Controller/Map/mapcontroller.h"
#include "Controller/Points/pointscontroller.h"
#include "Controller/Robots/robotscontroller.h"
#include "Model/Points/points.h"
#include "Model/Points/point.h"
#include "Model/Robots/robot.h"
#include "View/Points/pointview.h"
#include "View/Map/mapview.h"

PathPainter::PathPainter(MainWindow* const &mainWindow)
    : QGraphicsPathItem(mainWindow->getMapController()->getMapView()), pathDeleted(false), visiblePath(""), robotName(""){

    setPen(QPen(Qt::red));
    connect(this, SIGNAL(updatePoints(int, QString)), mainWindow->getPointsController(), SLOT(replacePoint(int, QString)));
}

void PathPainter::resetPathSlot(QSharedPointer<Points> points){
    path = QPainterPath();
    visiblePath = "";
    robotName = "";
    points->setPixmapAll(PointView::PixmapType::NORMAL);

    if(QSharedPointer<QVector<QSharedPointer<PointView>>> group = points->getGroups()->value(PATH_GROUP_NAME)){
        for(int i = 0; i < group->size(); i++){
            group->at(i)->deleteLater();
        }
        group->clear();
    }

    currentPath.clear();
    setPath(path);
}

void PathPainter::addPathPoint(QSharedPointer<Points> points, QPointer<RobotView> robotView, GraphicItemState state, QString name, double x, double y, int waitTime){
    //qDebug() << "PathPainter::addPathPointSlot called" << name << x << y;

    if(currentPath.size() == 0 || (currentPath.size() > 0 && !currentPath.last()->getPoint().comparePos(x, y))){

        int nb = nbUsedPointView(name, x, y);
        QSharedPointer<PointView> pointView = points->findPathPointView(x, y);

        /// If found, it's a permanent point which is already in the path
        if(nb > 0 && pointView){
            points->addPoint(PATH_GROUP_NAME, pointView);
        } else {
            if(!points->isAPoint(name, x, y))
                name = PATH_POINT_NAME + QString::number(currentPath.size()+1);

            Point::PointType type = Point::PointType::PATH;

            if((robotView && robotView->getRobot()->getHome()
                && robotView->getRobot()->getHome()->getPoint()->getName().compare(name) == 0)
                    || (robotView == NULL && points->isAHome(name, x, y)))
                type = Point::PointType::HOME;

            points->addPoint(PATH_GROUP_NAME, name, x, y, true, type);
        }

        points->getGroups()->value(PATH_GROUP_NAME)->last()->setState(state);
        Point point = *(points->getGroups()->value(PATH_GROUP_NAME)->last()->getPoint());

        currentPath.push_back(QSharedPointer<PathPoint>(new PathPoint(point, waitTime)));

        /// Updates the path painter
        updatePathPainter(points, false);
    }
}

void PathPainter::orderPathPointChanged(QSharedPointer<Points> points, int from, int to){
    //qDebug() << "PathPainter::orderPathPointChangedSlot called from" << from << "to" << to;

    /// Do the change in the model
    qDebug() << currentPath.size();
    QSharedPointer<PathPoint> pathPoint = currentPath.takeAt(from);

    QSharedPointer<PointView> pointView = points->getGroups()->value(PATH_GROUP_NAME)->takeAt(from);

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

    updatePathPainterName(points);
    /// Update the path painter
    updatePathPainter(points, false);
}

void PathPainter::updatePathPainterName(QSharedPointer<Points> points){
    for(int i = 0; i < currentPath.size(); i++){
        if(currentPath.at(i)->getPoint().getName().contains(PATH_POINT_NAME)){
            Point point = currentPath.at(i)->getPoint();
            point.setName(PATH_POINT_NAME + QString::number(i+1));
            currentPath.at(i)->setPoint(point);
            points->getGroups()->value(PATH_GROUP_NAME)->at(i)->getPoint()->setName(point.getName());
        }
    }
}


void PathPainter::deletePathPoint(QSharedPointer<Points> points, int id){
    //qDebug() << "PathPainter::deletePathPointSlot called with id" << id << "current path size" << currentPath.size();
    /// Remove the item from the model
    currentPath.remove(id);
    points->getGroups()->value(PATH_GROUP_NAME)->remove(id);

    /// updates the tooltips
    updateCurrentPath(points);

    visiblePath = "";

    /// Updates the path painter
    updatePathPainter(points, false);
}

void PathPainter::editPathPoint(const QSharedPointer<Points> points, int id, QString name){

    qDebug() << "size before edit" << currentPath.size();
    QSharedPointer<PointView> newPointView = points->getGroups()->value(points->findPointIndexes(name).first)->
            takeAt(points->findPointIndexes(name).second);
    /// If found, it's a permanent point else it's a temporary point
    if(newPointView){
        points->getGroups()->value(PATH_GROUP_NAME)->remove(id);
        points->insertPoint(PATH_GROUP_NAME, id, newPointView);
        currentPath.at(id)->setPoint(*(newPointView->getPoint()));
    } else
        qDebug() << "PathPainter::editPathPoint editing a tmpPoint";

    /// Update the path painter
    updatePathPainter(points, false);
    qDebug() << "size after edit" << currentPath.size();
}

void PathPainter::actionChangedSlot(int id, QString waitTimeStr){
    //qDebug() << "PathPainter::actionChangedSlot called" << id << waitTimeStr;
    int waitTime = waitTimeStr.toInt();
    currentPath.at(id)->setWaitTime(waitTime);
}

void PathPainter::updateCurrentPath(QSharedPointer<Points> points){
    qDebug() << "updatecurrentpath called";
    /// to update correctly the tooltips of path points which are not permanent points
    /// for example if one point is deleted or if a point which used to be traversed twice is now only used once
    unsigned int number(1);
    for(int i = 0; i < currentPath.size(); i++){
        currentPath.at(i)->setPoint(*(points->getGroups()->value(PATH_GROUP_NAME)->at(i)->getPoint()));
        if(currentPath.at(i)->getPoint().getName().contains("PathPoint")){
            /// updates the name of the points of the path known to the path painter
            currentPath.at(i)->getPoint().setName("PathPoint" + QString::number(number));
            /// updates the name of the points inside the model in order to be consistent with the path
            points->findPathPointView(currentPath.at(i)->getPoint().getPosition().getX(),
                                      currentPath.at(i)->getPoint().getPosition().getY())
                    ->getPoint()->setName("PathPoint" + QString::number(number));
            /// updates the tooltips using the updated names
            points->findPathPointView(currentPath.at(i)->getPoint().getPosition().getX(),
                                      currentPath.at(i)->getPoint().getPosition().getY())
                    ->setToolTip("PathPoint" + QString::number(number));
            number++;
        }
    }
}

void PathPainter::updatePathPainter(const QSharedPointer<Points> points, const bool savePath /* to deal with pv selected after save */){
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    QSharedPointer<QVector<QSharedPointer<PointView>>> group = points->getGroups()->value(PATH_GROUP_NAME);

    QSharedPointer<PointView> startPointView(0);
    QSharedPointer<PointView> endPointView(0);
    QSharedPointer<PointView> currentPointView(0);

    if(group && group->size() > 0){
        for(int i = 0; i < group->size(); i++){
            currentPointView = group->at(i);
            QPointF pointCoord = QPointF(currentPointView->getPoint()->getPosition().getX(),
                                         currentPointView->getPoint()->getPosition().getY());

            if(i == 0){
                path = QPainterPath(pointCoord);
                startPointView = currentPointView;
            } else {
                path.lineTo(pointCoord);
                if(currentPointView->getType() != PointView::PixmapType::SELECTED || savePath)
                    currentPointView->setPixmap(PointView::PixmapType::MID);
            }

            if(i == group->size()-1)
                endPointView = currentPointView;
        }

        setPath(path);

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            if(startPointView->getType() != PointView::PixmapType::SELECTED || savePath)
                startPointView->setPixmap(PointView::PixmapType::START_STOP);

        } else {
            if(startPointView->getType() != PointView::PixmapType::SELECTED || savePath)
                startPointView->setPixmap(PointView::PixmapType::START);

            if(endPointView->getType() != PointView::PixmapType::SELECTED || savePath)
                endPointView->setPixmap(PointView::PixmapType::STOP);
        }
    } else {
        resetPathSlot(points);
    }
}

void PathPainter::updatePathPainterPointViewSlot(const QSharedPointer<QVector<QSharedPointer<PointView>>> group){
    //qDebug() << "PathPainter::updatePathPainterPointViewSlot called";

    QSharedPointer<PointView> startPointView(0);
    QSharedPointer<PointView> endPointView(0);
    QSharedPointer<PointView> currentPointView(0);

    if(group && group->size() > 0){
        for(int i = 0; i < group->size(); i++){
            currentPointView = group->at(i);

            if(i == 0){
                startPointView = currentPointView;
            } else {
                if(currentPointView->getType() != PointView::PixmapType::SELECTED)
                    currentPointView->setPixmap(PointView::PixmapType::MID);
            }

            if(i == group->size()-1)
                endPointView = currentPointView;
        }

        if(*(startPointView->getPoint()) == *(endPointView->getPoint())){
            if(startPointView->getType() != PointView::PixmapType::SELECTED)
                startPointView->setPixmap(PointView::PixmapType::START_STOP);
        } else {
            if(startPointView->getType() != PointView::PixmapType::SELECTED)
                startPointView->setPixmap(PointView::PixmapType::START);

            if(endPointView->getType() != PointView::PixmapType::SELECTED)
                endPointView->setPixmap(PointView::PixmapType::STOP);
        }
    }
}

int PathPainter::nbUsedPointView(QString name, double x, double y){
    //qDebug() << "PathPainter::nbUsedPointView called";
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

void PathPainter::setCurrentPath(const QSharedPointer<Points> points, const QPointer<RobotView> robotView, const GraphicItemState state, const QVector<QSharedPointer<PathPoint>>& _currentPath, QString pathName){
    resetPathSlot(points);
    visiblePath = pathName;

    if(robotView)
        robotName = robotView->getRobot()->getName();

    for(int i = 0; i < _currentPath.size(); i++){
        Point point = _currentPath.at(i)->getPoint();
        addPathPoint(points, robotView, state, point.getName(), point.getPosition().getX(), point.getPosition().getY(), _currentPath.at(i)->getWaitTime());
    }
}
