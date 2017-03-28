#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

class CmdRobotWorker;
class RobotPositionWorker;
class MetadataWorker;
class SendNewMapWorker;
class LocalMapWorker;
class ScanMapWorker;
class TeleopWorker;
class ParticleCloudWorker;
class CommandController;

#include <QObject>
#include <QThread>
#include <QPointer>
#include <QImage>

class RobotController : public QObject {
    Q_OBJECT
public:
    RobotController(QObject* parent, QString ip);
    ~RobotController();
    void stopThreads();

private:
    /**
     * @brief launchWorkers
     * @param mainWindow
     * connects the workers in order to receive map, robot position and laser data
     */
    void launchWorkers();

private slots:
    void mapReceivedSlot(QByteArray, int, QString, QString, QString, QString, QString, QString, int, int);
    void sendNewMapToRobots(QString);
    void doneSendingMapSlot();
    void updateMetadata(int width, int height, float resolution, float originX, float originY);
    void updateRobot(float posX, float posY, float ori);
    void robotIsDeadSlot();
    void updateRobotInfo(QString robotInfo);
    void portSentSlot();

signals:
    void robotIsDead(QString);
    void pingSignal();
    void sendNewMapSignal(QString mapId, QString date, QString metadata, QImage map);
    void teleopCmd(int);
    void newRobotPos(QString ip, float posX, float posY, float ori);
    void newMetadata(int width, int height, float resolution, float originX, float originY);
    void updatePath(QString ip, QStringList strList);
    void updateHome(QString ip, QString homeName, float homeX, float homeY);

    void stopCmdRobotWorker();
    void stopRobotWorker();
    void stopMetadataWorker();
    void stopNewMapWorker();
    void stopLocalMapWorker();
    void stopMapWorker();
    void stopTeleopWorker();
    void stopParticleCloudWorker();

    void startCmdRobotWorker();
    void startRobotWorker();
    void startMetadataWorker();
    void startNewMapWorker();
    void startLocalMapWorker();
    void startMapWorker();
    void startTeleopWorker();
    void startParticleCloudWorker();

private:
    QString ip;
    QPointer<CommandController> commandController;

    QPointer<CmdRobotWorker> cmdRobotWorker;
    QPointer<RobotPositionWorker> robotWorker;
    QPointer<MetadataWorker> metadataWorker;
    QPointer<SendNewMapWorker> newMapWorker;
    QPointer<LocalMapWorker> localMapWorker;
    QPointer<ScanMapWorker> mapWorker;
    QPointer<TeleopWorker> teleopWorker;
    QPointer<ParticleCloudWorker> particleCloudWorker;

    QThread cmdThread;
    QThread robotThread;
    QThread metadataThread;
    QThread newMapThread;
    QThread localMapThread;
    QThread mapThread;
    QThread teleopThread;
    QThread particleCloudThread;
};

#endif // ROBOTCONTROLLER_H
