#include "drawobstacles.h"
#include <QDebug>
#include <QPainter>
#include <assert.h>
#include "View/mapview.h"
#include "Model/robots.h"
#include "Controller/mainwindow.h"
#include "Model/robot.h"
#include <QtMath>

DrawObstacles::DrawObstacles(const QSize _size, QSharedPointer<Robots> _robots, QGraphicsItem *parent) : QGraphicsItem(parent), size(_size), robots(_robots)
{}

/// the event that is called when calling update() and which is drawing on the map
void DrawObstacles::paint(QPainter *_painter, const QStyleOptionGraphicsItem *, QWidget *){
    _painter->setPen(Qt::red);
    /// iterator on our map of obstacles
    QMapIterator<QString, QPair<QVector<QPointF>, bool>> it_obs(obstacles);
    /// the iterator is iterating over the IP addresses of the map, each of them
    /// representing a robot
    while(it_obs.hasNext()){
        it_obs.next();
        QVector<QPointF> curr_obstacles = it_obs.value().first;
        for(int i = 0; i < curr_obstacles.size(); i++)
            _painter->drawPoint(curr_obstacles.at(i));
    }
}

void DrawObstacles::drawObstacles(float angle_min, float angle_max, float angle_increment, const QVector<float>& ranges, QString ipAddress){
    qDebug() << "MainWindow::drawObstacles called" << angle_min << angle_max << angle_increment;
    /// if the IP address is in the map we update the obstacles corresponding to this entry
    /// the second condition checks that the full scan has been received
    if(obstacles.find(ipAddress) != obstacles.end() && static_cast<int>((angle_max-angle_min)/angle_increment) <= ranges.size()){
        QVector<QPointF> points = convertRangesToPoints(angle_min, angle_increment, ranges, ipAddress);
        obstacles.insert(ipAddress, QPair<QVector<QPointF>, bool>(points, true));
    }
    /// draws on the map by calling paint
    update();
}

void DrawObstacles::addNewRobotObstacles(QString ipAddress){
    qDebug() << "MainWindow::addNewRobotObstacles called with IP" << ipAddress;
    obstacles.insert(ipAddress, QPair<QVector<QPointF>, bool>(QVector<QPointF>(), true));
}

QVector<QPointF> DrawObstacles::convertRangesToPoints(const float angle_min /* rad */, const float angle_increment /* rad */, const QVector<float> ranges, const QString ipAddress) const {
    qDebug() << "MainWindow::convertRangesToPoints called with" << ranges.size() << "values";
    QVector<QPointF> points;
    QPointer<Robot> robot = robots->getRobotViewByIp(ipAddress)->getRobot();
    for(int i = 0; i < ranges.size(); i++)
        ///  / by 0.05 = resolution -> * 20 is faster
        points.push_back(QPointF(robot->getPosition().getX() + (ranges.at(i) * cos(angle_min + i*angle_increment)) * 20 ,
                                 robot->getPosition().getY() + (ranges.at(i) * sin(angle_min + i*angle_increment)) * 20));

    return points;
}

void DrawObstacles::removeRobotObstacles(const QString ipAddress){
    assert(obstacles.remove(ipAddress));
    /// have to repaint after removing the obstacles
    update();
}

QRectF DrawObstacles::boundingRect() const {
    /// that's so that when you zoom your points don't disappear
    return QRectF(0, 0, 5 * size.width(), 5 * size.height());
}

void DrawObstacles::turnOnLaserFeedBack(const QString ipAddress, const bool activate){
    qDebug() << "DrawObstacles::turnOnLaserFeedBack called";
    (activate) ? addNewRobotObstacles(ipAddress) : removeRobotObstacles(ipAddress);
}
