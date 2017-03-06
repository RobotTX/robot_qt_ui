#ifndef POINTCONTROLLER_H
#define POINTCONTROLLER_H

class Points;

#include <QObject>

class PointController : public QObject {
    Q_OBJECT
public:
    PointController(QObject *parent, QString mapFile);

private:
    void loadPoints(QString fileName);

private:
    Points* points;
};

#endif // POINTCONTROLLER_H
