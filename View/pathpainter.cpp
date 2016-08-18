#include "pathpainter.h"
#include "View/mapview.h"
#include "Model/points.h"
#include "Model/point.h"
#include "Model/robot.h"
#include "View/pointview.h"
#include <QDebug>
#include <QPainterPath>
#include <QPixmap>
#include "Controller/mainwindow.h"

PathPainter::PathPainter(MainWindow* const &mainWindow, MapView* const &mapPixmapItem, QSharedPointer<Points> _points)
    : QGraphicsPathItem(mapPixmapItem), points(_points), mainWindow(mainWindow), mapView(mapPixmapItem), pathDeleted(false){
    setPen(QPen(Qt::red));
}

void PathPainter::resetPathSlot(void){
    qDebug() << "PathPainter::resetPathSlot called";
    path = QPainterPath();
    points->setPixmapAll(PointView::PixmapType::NORMAL, mainWindow->getSelectedRobot());

    if(QSharedPointer<QVector<QSharedPointer<PointView>>> group = points->getGroups()->value(PATH_GROUP_NAME)){
        for(int i = 0; i < group->size(); i++){
            group->at(i)->deleteLater();
        }
        group->clear();
    }

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
    QSharedPointer<PointView> pointView = points->findPathPointView(x, y);

    /// If found, it's a permanent point which is already in the path
    if(nb > 0 && pointView){
        points->addPoint(PATH_GROUP_NAME, pointView);
    } else {
        if(!points->isAPoint(name, x, y))
            name = PATH_POINT_NAME + QString::number(currentPath.size()+1);

        Point::PointType type = Point::PointType::PATH;
        qDebug() << "PathPainter::addPathPointSlot" << (mainWindow->getSelectedRobot() == NULL);
        if((mainWindow->getSelectedRobot() && mainWindow->getSelectedRobot()->getRobot()->getHome() && mainWindow->getSelectedRobot()->getRobot()->getHome()->getPoint()->getName().compare(name) == 0)
                || (mainWindow->getSelectedRobot() == NULL))
            type= Point::PointType::HOME;
        /*if(points->isAHome(name, x, y))
            type= Point::PointType::HOME;*/
        points->addPoint(PATH_GROUP_NAME, name, x, y, true, type, mapView, mainWindow);
    }

    points->getGroups()->value(PATH_GROUP_NAME)->last()->setState(mapView->getState());
    Point point = *(points->getGroups()->value(PATH_GROUP_NAME)->last()->getPoint());
    currentPath.push_back(QSharedPointer<PathPoint>(new PathPoint(point, PathPoint::Action::WAIT)));

    /// Update the path painter
    updatePathPainterSlot();
}

void PathPainter::orderPathPointChangedSlot(int from, int to){
    qDebug() << "PathPainter::orderPathPointChangedSlot called from" << from << "to" << to;

    /// Do the change in the model
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

    updatePathPainterName();
    /// Update the path painter
    updatePathPainterSlot();
}

void PathPainter::updatePathPainterName(void){
    qDebug() << "PathPainter::updatePathPainterName called";
    for(int i = 0; i < currentPath.size(); i++){
        if(currentPath.at(i)->getPoint().getName().contains(PATH_POINT_NAME)){
            Point point = currentPath.at(i)->getPoint();
            point.setName(PATH_POINT_NAME + QString::number(i+1));
            currentPath.at(i)->setPoint(point);
            points->getGroups()->value(PATH_GROUP_NAME)->at(i)->getPoint()->setName(point.getName());
        }
    }
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

    QSharedPointer<PointView> newPointView = points->getGroups()->value(PATH_GROUP_NAME)->takeAt(id);

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

void PathPainter::actionChangedSlot(int id, int action, QString waitTimeStr){
    qDebug() << "PathPainter::actionChangedSlot called" << id << waitTimeStr;
    int waitTime = waitTimeStr.toInt();
    currentPath.at(id)->setAction(static_cast<PathPoint::Action>(action));
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
    points->setPixmapAll(PointView::PixmapType::NORMAL, mainWindow->getSelectedRobot());
    QSharedPointer<QVector<QSharedPointer<PointView>>> group = points->getGroups()->value(PATH_GROUP_NAME);

    QSharedPointer<PointView> startPointView(0);
    QSharedPointer<PointView> endPointView(0);
    QSharedPointer<PointView> currentPointView(0);

    if(group && group->size() > 0){
        for(int i = 0; i < group->size(); i++){
            currentPointView = group->at(i);
            QPointF pointCoord = QPointF(currentPointView->getPoint()->getPosition().getX(),
                                         currentPointView->getPoint()->getPosition().getY());

            /// TODO if the original point is displaying a home => display home else no display home
            /// ( changer type ? home -> path et path -> home ? )

            /*QSharedPointer<PointView> realPointView = points->findPointView(currentPointView->getPoint()->getName());
            qDebug() << "realPointView" << (realPointView == NULL);

            //if(realPointView->getType() == )*/
            if(i == 0){
                path = QPainterPath(pointCoord);
                startPointView = currentPointView;
            } else {
                path.lineTo(pointCoord);
                currentPointView->setPixmap(PointView::PixmapType::MID);
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
    QSharedPointer<QVector<QSharedPointer<PointView>>> group = points->getGroups()->value(PATH_GROUP_NAME);

    QSharedPointer<PointView> startPointView(0);
    QSharedPointer<PointView> endPointView(0);
    QSharedPointer<PointView> currentPointView(0);

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

void PathPainter::setCurrentPath(const QVector<QSharedPointer<PathPoint>> _currentPath){
    resetPathSlot();
    for(int i = 0; i < _currentPath.size(); i++){
        Point point = _currentPath.at(i)->getPoint();
        addPathPointSlot(point.getName(), point.getPosition().getX(), point.getPosition().getY());
    }
}

void PathPainter::setOldPath(const QVector<QSharedPointer<PathPoint> > _oldPath){
    oldPath.clear();
    for(int i = 0; i < _oldPath.size(); i++){
        QSharedPointer<PathPoint> pathPoint = QSharedPointer<PathPoint>(new PathPoint(_oldPath.at(i)->getPoint(), _oldPath.at(i)->getAction(), _oldPath.at(i)->getWaitTime()));
        oldPath.push_back(pathPoint);
    }
}

void PathPainter::clearOldPath(){
    oldPath.clear();
}
