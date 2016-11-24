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
{
    /// To have this QGraphicsLayer above the map
    //sepixmapItem->setFlag(QGraphicsItem::ItemStacksBehindParent);tZValue(2);

}


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
    qDebug() << "MainWindow::drawObstacles called" << angle_min << angle_max << angle_increment;
    qDebug() << "Number of values received from the laser" << ranges.size() << "at" << ipAddress;
    /// if the IP address is in the map we update the obstacles corresponding to this entry
    if(obstacles.find(ipAddress) != obstacles.end()){
        QVector<QPointF> points = convertRangesToPoints(angle_min, angle_increment, ranges, ipAddress);
        obstacles.insert(ipAddress, points);
    }
    update();
}

void DrawObstacles::addNewRobotObstacles(QString ipAddress){
    qDebug() << "MainWindow::addNewRobotObstacles called with IP" << ipAddress;
    obstacles.insert(ipAddress, QVector<QPointF>());
}

QVector<QPointF> DrawObstacles::convertRangesToPoints(const float angle_min /* rad */, const float angle_increment /* rad */, const QVector<float> ranges, const QString ipAddress) const {
    qDebug() << "MainWindow::convertRangesToPoints called with" << ranges.size() << "values";
    QVector<QPointF> points;
    QPointer<Robot> robot = robots->getRobotViewByIp(ipAddress)->getRobot();
    qDebug() << "Robot position " << robot->getPosition().getX() <<
                robot->getPosition().getY() <<
                robot->getOrientation();
    for(int i = 0; i < ranges.size(); i++){
        ///  / by 0.05 = resolution -> * 20 is faster
        /// don't forget orientation of the robot given in degrees qDegreesToRadians(robot->getOrientation())
        points.push_back(QPointF(robot->getPosition().getX() + (ranges.at(i) * cos(angle_min + i*angle_increment)) * 20 ,
                                 robot->getPosition().getY() + (ranges.at(i) * sin(angle_min + i*angle_increment)) * 20));
    }
    return points;
}

void DrawObstacles::removeRobotObstacles(const QString ipAddress){
    assert(obstacles.remove(ipAddress));
    update();
}

QRectF DrawObstacles::boundingRect() const {
    /// that's so that when you zoom your points don't disappear
    return QRectF(0, 0, 5 * size.width(), 5 * size.height());
}
