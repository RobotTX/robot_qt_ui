#ifndef DRAWOBSTACLESTASK_H
#define DRAWOBSTACLESTASK_H

#include <QRunnable>
#include <QMap>
#include <QVector>
#include <QPointF>
#include <QPointer>
#include "Model/robot.h"

class DrawObstaclesTask : public QRunnable
{
public:
    DrawObstaclesTask(const float angle_min, const float angle_max, const float angle_increment,
                      const QVector<float>& ranges, const QString ipAddress, QMap<QString, QVector<QPointF> >* obstacles, QPointer<Robot> _robot);

    ~DrawObstaclesTask();

    QVector<QPointF> convertRangesToPoints(const float angle_min /* rad */, const float angle_increment /* rad */, const QVector<float> ranges) const ;

    void run();

private:
    float angle_min;
    float angle_max;
    float angle_increment;
    QVector<float> ranges;
    QString ipAddress;
    QMap<QString, QVector<QPointF> > *obstacles;
    QPointer<Robot> robot;
};

#endif /// DRAWOBSTACLESTASK_H
