#ifndef MAINMENUCONTROLLER_H
#define MAINMENUCONTROLLER_H

class QQmlApplicationEngine;

#include <QObject>

class MainMenuController : public QObject {
    Q_OBJECT
public:
    MainMenuController(QObject* parent = Q_NULLPTR);

};

#endif // MAINMENUCONTROLLER_H
