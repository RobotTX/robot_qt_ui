#include "authentification.h"
#include <QApplication>
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include <QImage>
#include <QGuiApplication>
#include <QtQuick/QQuickView>
#include "Controller/maincontroller.h"
#include "Controller/authentification.h"
#include "View/EditMap/editmappainteditem.h"
#include "View/Robot/scanmappainteditem.h"
#include "View/Robot/obstaclespainteditem.h"

Authentification::Authentification(QQmlApplicationEngine *engine, QQuickWindow* _window, QObject* parent) : QObject(parent), window(_window)
{
    qDebug("object is being created");
    QList<QObject*> qmlList = engine->rootObjects();

    if(qmlList.size() == 1) {

        QObject *applicationWindow = qmlList.at(0);
        QObject* loginBtn = applicationWindow->findChild<QObject*>("loginBtn");
        if(loginBtn){
            connect(loginBtn, SIGNAL(checkLogin()), this, SLOT(checkLoginSlot()));
        } else {
            /// NOTE can probably remove that when testing phase is over
            qDebug() << "Authentification::Authentification could not find the loginBtn";
            Q_UNREACHABLE();
        }

    }
}

Authentification::~Authentification() {
    qDebug() << "Object is being deleted";
}

void Authentification::checkLoginSlot() {
    // main application
    mainEngine.load(QUrl("qrc:/main.qml"));
    qDebug("mainEngine from mainController opening");
    QQuickWindow *applicationWindow = qobject_cast<QQuickWindow*>(mainEngine.rootObjects().at(0));
    if (!applicationWindow) {
        qFatal("Error: Your root item has to be a window.");
        Q_UNREACHABLE();
    } else {
        mainController = new MainController(&mainEngine);
        connect(mainController,SIGNAL(deco()),this,SLOT(deconnexionAuth()));
        window->hide();
    }
}

void Authentification::deconnexionAuth() {
    qDebug("deconnexion");
    QQuickWindow *applicationWindow = qobject_cast<QQuickWindow*>(mainEngine.rootObjects().at(0));
    if (!applicationWindow) {
        qFatal("Error: Your root item has to be a window.");
        Q_UNREACHABLE();
    } else {
        applicationWindow->close();
//        delete mainController;
        qDebug("applicationWindow closed");
        window->show();

    }
}

