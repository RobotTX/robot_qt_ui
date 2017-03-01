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

private slots:
    void menuClicked(QString txt, bool checked);

private:
    /// The main element containing all the QML elements
    QQmlApplicationEngine* engine;

    /// Controllers
    MainMenuController* mainMenuController;
};

#endif // MAINCONTROLLER_H
