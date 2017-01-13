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

DrawObstacles::DrawObstacles(QSharedPointer<Robots> _robots, QObject *parent)
    : QObject(parent), robots(_robots){}

void DrawObstacles::drawObstacles(float angle_min, float angle_max, float angle_increment, const QVector<float>& ranges, QString ipAddress){
    //// might use something like that to scale up in the event where we would have to process the data of a lot of lasers

    //qDebug() << "DrawObstacles::drawObstacles called" << angle_min << angle_max << angle_increment;
    /// if the IP address is in the map we update the obstacles corresponding to this entry
    /// the second condition checks that the full scan has been received
    if(static_cast<int>((angle_max-angle_min)/angle_increment) <= ranges.size()){
        QPointer<RobotView> robotView = robots->getRobotViewByIp(ipAddress);
        if(robotView && robotView->getRobot()){
            QVector<QPointF> points = convertRangesToPoints(angle_min, angle_increment, ranges, robotView->getRobot());
            robotView->setObstacles(points);
        }
    }
}

QVector<QPointF> DrawObstacles::convertRangesToPoints(const float angle_min /* rad */, const float angle_increment /* rad */, const QVector<float> ranges, QPointer<Robot> robot) const {
    //qDebug() << "DrawObstacles::convertRangesToPoints called with" << ranges.size() << "values";
    QVector<QPointF> points;
    int i(ranges.size()-1);

    /// for improved performance
    //qDebug() << "DrawObstacles::convertRangesToPoints called with robot orientation " << robot->getOrientation();
    std::for_each(ranges.begin(), ranges.end(), [&](const float range) {
        points.push_back(QPointF(robot->getPosition().getX() + (range * cos(robot->getOrientation()*3.14159/180 - 3.14159/2 + angle_min + i*angle_increment)) * 20 ,
                                 robot->getPosition().getY() + (range * sin(robot->getOrientation()*3.14159/180 - 3.14159/2 + angle_min + i*angle_increment)) * 20)); i--; });
    return points;
}
