#ifndef ROBOTSCONTROLLER_H
#define ROBOTSCONTROLLER_H

class RobotController;
class MainController;
class RobotServerWorker;

#include <QObject>
#include <QPointer>
#include <QMap>
#include <QThread>
#include <QVariant>

class RobotsController : public QObject {
    Q_OBJECT
public:
    RobotsController(QObject *applicationWindow, MainController *parent);
    ~RobotsController();

    void setRobotPos(QString ip, float posX, float posY, float ori);

private:
    void launchServer();

private slots:
    void robotIsAliveSlot(QString name, QString ip, QString ssid, int stage, int battery);
    void robotIsDeadSlot(QString ip);
    void shortcutAddRobot();
    void shortcutDeleteRobot();
    void newRobotPosSlot(QString ip, float posX, float posY, float ori);
    void newMetadataSlot(int width, int height, float resolution, float originX, float originY);
    void updatePathSlot(QString ip, QStringList strList);
    void updateHomeSlot(QString ip, QString homeName, float homeX, float homeY);

signals:
    void stopRobotServerWorker();
    void addRobot(QVariant name, QVariant ip, QVariant ssid, QVariant stage, QVariant battery);
    void removeRobot(QVariant ip);
    void setPos(QVariant ip, QVariant posX, QVariant posY, QVariant orientation);
    void setHome(QVariant ip, QVariant name, QVariant posX, QVariant posY);
    void setPath(QVariant ip, QVariant name);
    void setPlayingPath(QVariant, QVariant);
    void addPathPoint(QVariant ip, QVariant name, QVariant posX, QVariant posY, QVariant waitTime);
    void displayRobots();
    void setStage(QVariant, QVariant);
    void setBattery(QVariant, QVariant);
    void newRobotPos(QString ip, float posX, float posY, float ori);
    void newMetadata(int width, int height, float resolution, float originX, float originY);
    void updatePath(QString ip, QStringList strList);
    void updateHome(QString ip, QString homeName, float homeX, float homeY);

private:
    QMap<QString, QPointer<RobotController>> robots;

    RobotServerWorker* robotServerWorker;
    QThread serverThread;
};

#endif // ROBOTSCONTROLLER_H
