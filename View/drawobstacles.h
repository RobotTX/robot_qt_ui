#ifndef DRAWOBSTACLES_H
#define DRAWOBSTACLES_H

class Robot;

#include <QObject>
#include "Model/robots.h"

class DrawObstacles : public QObject {
    Q_OBJECT

public:
    DrawObstacles(QSharedPointer<Robots> _robots, QObject *parent = Q_NULLPTR);

    QVector<QPointF> convertRangesToPoints(const float angle_min, const float angle_increment, const QVector<float> ranges, QPointer<Robot> robot) const;

private slots:
    void drawObstacles(float angle_min, float angle_max, float angle_increment, const QVector<float> &ranges, QString ipAddress);

private:
    QSharedPointer<Robots> robots;
};

#endif /// DRAWOBSTACLES_H
