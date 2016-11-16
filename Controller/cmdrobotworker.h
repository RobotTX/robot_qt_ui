#ifndef CMDROBOTWORKER_H
#define CMDROBOTWORKER_H


#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>
#include <QTimer>

#define ROBOT_TIMER 15

/**
 * @brief The CmdRobotWorker class
 * The worker connects to the robot at the given ipAddress & port to send command and receive messages to/from the robot
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
    /**
     * @brief robotIsDead
     * @param hostname
     * @param ip
     * Send a signal to mainWindow when a robot die, (by disconnection or no ping within ROBOT_TIMER seconds
     */
    void robotIsDead(QString hostname, QString ip);

    /**
     * @brief cmdAnswer
     * @param answer
     * Send a ginal to the mainWindow that we received an answer from the robot
     * (usually after sending a command, the robot tell us if it succeeded or failed)
     */
    void cmdAnswer(QString answer);

    /**
     * @brief portSent
     * Send a signal to Robot when we succesfully sent the port we use for the other sockets
     * so that we start the other workers
     */
    void portSent();

private slots:
    /**
     * @brief connectSocket
     * Called to start the connection with the robot
     */

    void connectSocket();
    /**
     * @brief connectedSlot
     * Slot called when we are connected to the host
     */
    void connectedSlot();

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

    /**
     * @brief changeRobotNameSlot
     * @param name
     * When we changed the name of the robot
     */
    void changeRobotNameSlot(QString name);

    /**
     * @brief sendCommand
     * @param cmd
     * @return
     * Called when we want to send a command
     */
    void sendCommand(const QString cmd);

    /**
     * @brief pingSlot
     * When we receive a ping from the robot, we reset the timer
     */
    void pingSlot(void);

    /**
     * @brief timerSlot
     * Called every second to check for how long we haven't receive any ping
     */
    void timerSlot(void);

    /**
     * @brief stopWorker
     * Slot to stop the worker
     */
    void stopWorker();

    /**
     * @brief errorConnectionSlot
     * @param error
     * Called when an error occurs with the socket
     * When we try to connectToHost, if the socket on the robot is not open yet (or other problem),
     * we'll try to connect again in this slot
     */
    void errorConnectionSlot(QAbstractSocket::SocketError error);


private :
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    QString robotName;
    int metadataPort;
    int robotPort;
    int mapPort;
    QTimer* timer;
    int timeCounter;
};

#endif // CMDROBOTWORKER_H
