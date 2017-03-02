#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

class QQmlApplicationEngine;
class MainMenuController;

#include <QObject>
#include <QList>

class MainController : public QObject {
    Q_OBJECT
public:
    MainController(QQmlApplicationEngine* _engine, QObject* parent = Q_NULLPTR);
};

#endif // MAINCONTROLLER_H
