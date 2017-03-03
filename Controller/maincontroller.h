#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

class QQmlApplicationEngine;
class MainMenuController;
class MapController;

#include <QObject>
#include <QList>

class MainController : public QObject {
    Q_OBJECT
public:
    MainController(QQmlApplicationEngine* _engine, QObject* parent = Q_NULLPTR);

private:
    MainMenuController* mainMenuController;
    MapController* mapController;
};

#endif // MAINCONTROLLER_H
