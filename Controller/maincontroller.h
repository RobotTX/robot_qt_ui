#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

class QQmlApplicationEngine;
class MainMenuController;
class MapController;
class PointController;

#include <QObject>
#include <QList>

class MainController : public QObject {
    Q_OBJECT
public:
    MainController(QQmlApplicationEngine* _engine, QObject* parent = Q_NULLPTR);

private slots:
    void checkPoint(QString name, QString oldName, double x, double y);
    void saveMapConfig(QString fileName, double zoom, double centerX, double centerY) const;

private:
    MainMenuController* mainMenuController;
    MapController* mapController;
    PointController* pointController;
};

#endif /// MAINCONTROLLER_H
