#ifndef CMDROBOTWORKER_H
#define CMDROBOTWORKER_H

class QTimer;

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QSharedPointer>

#define ROBOT_TIMER 15

/**
 * @brief The CmdRobotWorker class
 * The thread connects to the robot at the given ipAddress & port to receive the map the robot
 * is scanning
 */
class CmdRobotWorker : public QObject {
    Q_OBJECT
public:
    /**
     * @brief CmdRobotWorker
     * @param ipAddress
     * @param port
     */
    CmdRobotWorker(const QString ipAddress, const int cmdPort, const int metadataPort, const int robotPort, const int mapPort, const QString _robotName);
    ~CmdRobotWorker();

signals:
    void robotIsDead(QString hostname, QString ip);
    void cmdAnswer(QString answer);
    void portSent();

private slots:
    void connectSocket();
    /**
     * @brief connectedSlot
     * Slot called when we are connected to the host
     */
    void connectedSlot();

    /**
     * @brief errorSlot
     * @param error
     * (Not connected) Slot called when there is an error
     */
    void errorSlot(QAbstractSocket::SocketError error);
    void onStateChanged(QAbstractSocket::SocketState error);

    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();

    /**
     * @brief readTcpDataSlot
     * Read the data we receive
     */
    void readTcpDataSlot();
    void changeRobotNameSlot(QString name);

    /**
     * @brief sendCommand
     * @param cmd
     * @return
     * Called when we want to send a command
     */
    void sendCommand(const QString cmd);
    void pingSlot(void);
    void timerSlot(void);
    void stopCmdRobotWorkerSlot();


private :
    QSharedPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    QString robotName;
    bool connected;
    int metadataPort;
    int robotPort;
    int mapPort;
    bool stop;
    QTimer* timer;
    int timeCounter;
};

#endif // CMDROBOTWORKER_H
