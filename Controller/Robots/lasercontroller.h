#ifndef LASERCONTROLLER_H
#define LASERCONTROLLER_H

class Robot;

#include <QObject>
#include "Model/Robots/robots.h"

class LaserController :public QObject {

    Q_OBJECT

public:

    LaserController(QSharedPointer<Robots> _robots, QObject* parent);

    QVector<QPointF> convertRangesToPoints(const float angle_min /* rad */, const float angle_increment /* rad */, const QVector<float> ranges, QPointer<Robot> robot) const;


private slots:
    void drawObstacles(float angle_min, float angle_max, float angle_increment, const QVector<float> &ranges, QString ipAddress);

private:
    QSharedPointer<Robots> robots;
};

#endif // LASERCONTROLLER_H
