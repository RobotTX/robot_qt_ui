#ifndef AUTHENTIFICATION_H
#define AUTHENTIFICATION_H
#include <QObject>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include "maincontroller.h"

class Authentification : public QObject {
    Q_OBJECT

public:
    Authentification(QQmlApplicationEngine* _engine, QQuickWindow *_window, QObject* parent = Q_NULLPTR);
    ~Authentification();

private slots:
    void checkLoginSlot();
    void deconnexionAuth();

private:
    QQmlApplicationEngine mainEngine;
    MainController* mainController;
    QQuickWindow* window;

};


#endif // AUTHENTIFICATION_H
