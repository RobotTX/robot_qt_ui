#include "drawobstaclestask.h"
#include <QThread>
#include <QDebug>

DrawObstaclesTask::DrawObstaclesTask(const float _angle_min, const float _angle_max, const float _angle_increment,
                                     const QVector<float> &_ranges, const QString _ipAddress, QMap<QString, QVector<QPointF> > *_obstacles, QPointer<Robot> _robot):
    angle_min(_angle_min), angle_max(_angle_max), angle_increment(_angle_increment), ranges(_ranges), ipAddress(_ipAddress), obstacles(_obstacles), robot(_robot)
{}

DrawObstaclesTask::~DrawObstaclesTask(){
    qDebug() << "Task working on robot" << robot->getName() << "finished : " << QThread::currentThreadId();
}

void DrawObstaclesTask::run(){
    /// if the IP address is in the map we update the obstacles corresponding to this entry
    /// the second condition checks that the full scan has been received
    if(obstacles->find(ipAddress) != obstacles->end() && static_cast<int>((angle_max-angle_min)/angle_increment) <= ranges.size()){
        QVector<QPointF> points = convertRangesToPoints(angle_min, angle_increment, ranges);
        obstacles->insert(ipAddress, points);
    }
}

QVector<QPointF> DrawObstaclesTask::convertRangesToPoints(const float angle_min /* rad */, const float angle_increment /* rad */, const QVector<float> ranges) const {
    qDebug() << "MainWindow::convertRangesToPoints called with" << ranges.size() << "values";
    QVector<QPointF> points;
    int i(ranges.size()-1);
    /// for improved performance
    std::for_each(ranges.begin(), ranges.end(), [&](const float range) { points.push_back(QPointF(robot->getPosition().getX() + (range * cos(angle_min + i*angle_increment)) * 20 ,
                                                                                                  robot->getPosition().getY() + (range  * sin(angle_min + i*angle_increment)) * 20)); i--; });
    return points;
}
