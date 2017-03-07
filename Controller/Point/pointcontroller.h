#ifndef POINTCONTROLLER_H
#define POINTCONTROLLER_H

class Points;

#include <QObject>

class PointController : public QObject {
    Q_OBJECT
public:
    PointController(QObject *applicationWindow, QString mapFile, QObject *parent);

private:
    void loadPoints(QString fileName);

private:
    Points* points;
};

#endif // POINTCONTROLLER_H
