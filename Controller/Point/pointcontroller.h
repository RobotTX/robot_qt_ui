#ifndef POINTCONTROLLER_H
#define POINTCONTROLLER_H

class Points;

#include <QObject>

class PointController : public QObject {
    Q_OBJECT
public:
    PointController(QObject *parent);

private:
    Points* points;
};

#endif // POINTCONTROLLER_H
