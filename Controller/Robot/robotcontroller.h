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
    void updateMetadata(int, int, float, float, float);
    void updateRobot(QString, float, float, float);
    void robotIsDeadSlot();
    void cmdAnswerSlot(QString);
    void updateRobotInfo(QString, QString);
    void portSentSlot();

signals:
    void robotIsDead(QString);
    void pingSignal();
    void sendCommandSignal(QString cmd);
    void sendNewMapSignal(QString mapId, QString date, QString metadata, QImage map);
    void teleopCmd(int);

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
