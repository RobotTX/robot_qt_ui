#include "drawobstacles.h"
#include <QDebug>
#include <QPainter>
#include <assert.h>
#include "View/mapview.h"
#include "Model/robots.h"
#include "Controller/mainwindow.h"
#include "Model/robot.h"
#include <QtMath>
#include <chrono>
#include <QThreadPool>
#include "drawobstaclestask.h"

DrawObstacles::DrawObstacles(const QSize _size, QSharedPointer<Robots> _robots, QGraphicsItem *parent) : QGraphicsItem(parent), size(_size), robots(_robots)
{}

/// the event that is called when calling update() and which is drawing on the map
void DrawObstacles::paint(QPainter *_painter, const QStyleOptionGraphicsItem *, QWidget *){
    _painter->setPen(Qt::red);
    /// iterator on our map of obstacles
    QMapIterator<QString, QVector<QPointF>> it_obs(obstacles);
    /// the iterator is iterating over the IP addresses of the map, each of them
    /// representing a robot
    while(it_obs.hasNext()){
        it_obs.next();
        QVector<QPointF> curr_obstacles = it_obs.value();
        for(int i = 0; i < curr_obstacles.size(); i++)
            _painter->drawPoint(curr_obstacles.at(i));
    }
}

void DrawObstacles::drawObstacles(float angle_min, float angle_max, float angle_increment, const QVector<float>& ranges, QString ipAddress){
    //// might use something like that to scale up in the event where we would have to process the data of a lot of lasers
    /*
    DrawObstaclesTask* t = new DrawObstaclesTask(angle_min, angle_max, angle_increment, ranges, ipAddress, &obstacles, robots->getRobotViewByIp(ipAddress)->getRobot());
    QThreadPool::globalInstance()->start(t, QThread::LowestPriority);
    */

    //qDebug() << "DrawObstacles::drawObstacles called" << angle_min << angle_max << angle_increment;
    /// if the IP address is in the map we update the obstacles corresponding to this entry
    /// the second condition checks that the full scan has been received
    if(obstacles.find(ipAddress) != obstacles.end() && static_cast<int>((angle_max-angle_min)/angle_increment) <= ranges.size()){
        QVector<QPointF> points = convertRangesToPoints(angle_min, angle_increment, ranges, ipAddress);
        obstacles.insert(ipAddress, points);
    }

    /// draws on the map by calling paint
    update();
}

void DrawObstacles::addNewRobotObstacles(QString ipAddress){
    qDebug() << "DrawObstacles::addNewRobotObstacles called with IP" << ipAddress;
    obstacles.insert(ipAddress, QVector<QPointF>());
}

QVector<QPointF> DrawObstacles::convertRangesToPoints(const float angle_min /* rad */, const float angle_increment /* rad */, const QVector<float> ranges, const QString ipAddress) const {
    //qDebug() << "DrawObstacles::convertRangesToPoints called with" << ranges.size() << "values";
    QVector<QPointF> points;
    QPointer<Robot> robot = robots->getRobotViewByIp(ipAddress)->getRobot();
    int i(ranges.size()-1);

    /// for improved performance
    //qDebug() << "DrawObstacles::convertRangesToPoints called with robot orientation " << robot->getOrientation();
    std::for_each(ranges.begin(), ranges.end(), [&](const float range) {
        points.push_back(QPointF(robot->getPosition().getX() + (range * cos(robot->getOrientation()*3.14159/180 - 3.14159/2 + angle_min + i*angle_increment)) * 20 ,
                                 robot->getPosition().getY() + (range * sin(robot->getOrientation()*3.14159/180 - 3.14159/2 + angle_min + i*angle_increment)) * 20)); i--; });
    return points;
}

void DrawObstacles::removeRobotObstacles(const QString ipAddress){
    assert(obstacles.remove(ipAddress));
    /// have to repaint after removing the obstacles
    update();
}

void DrawObstacles::clearRobotObstacles(const QString ipAddress){
    obstacles.insert(ipAddress, QVector<QPointF>());
    /// have to repaint after removing the obstacles
    update();
}

QRectF DrawObstacles::boundingRect() const {
    /// that's so that when you zoom your points don't disappear
    return QRectF(0, 0, 5 * size.width(), 5 * size.height());
}
