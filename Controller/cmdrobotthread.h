#ifndef CMDROBOTTHREAD_H
#define CMDROBOTTHREAD_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QSharedPointer>

#define MISSED_PING_TIMER 20

/**
 * @brief The CmdRobotThread class
 * The thread connects to the robot at the given ipAddress & port to receive the map the robot
 * is scanning
 */
class CmdRobotThread : public QThread {
    Q_OBJECT
public:
    /**
     * @brief CmdRobotThread
     * @param ipAddress
     * @param port
     */
    CmdRobotThread(const QString ipAddress, const int cmdPort, const int metadataPort, const int robotPort, const int mapPort, const QString _robotName, QObject *parent);

    /**
     * @brief run
     * Function called when we start a Thread
     */
    void run();
    QString waitAnswer();
    void resetCommandAnswer(){ commandAnswer = ""; }

    void delay(const int ms) const;
    bool isConnected() const {return connected;}

signals:
    void robotIsDead(QString hostname, QString ip);

private slots:
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
    void pingSlot();

private :
    QSharedPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    QString robotName;
    bool connected;
    QString commandAnswer;
    int missedPing;
    int metadataPort;
    int robotPort;
    int mapPort;
};

#endif // CMDROBOTTHREAD_H
