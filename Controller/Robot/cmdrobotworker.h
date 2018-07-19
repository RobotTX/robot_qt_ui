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
    CmdRobotWorker(const QString ipAddress, const int cmdPort, const int robotPort, const int mapPort, const int _laserPort, const int _mp3Port);
    ~CmdRobotWorker();

signals:
    /**
     * @brief robotIsDead
     * @param hostname
     * @param ip
     * Send a signal to mainWindow when a robot die, (by disconnection or no ping within ROBOT_TIMER seconds
     */
    void robotIsDead();

    /**
     * @brief cmdAnswer
     * @param answer
     * Send a ginal to the mainWindow that we received an answer from the robot
     * (usually after sending a command, the robot tell us if it succeeded or failed)
     */
    void cmdAnswer(QString answer);

    /**
     * @brief newConnection
     * @param ipAddress
     * @param home_pos
     * emitted when a robot just connected, the signal is caught in the main window
     * and calls a slot that will make sure the robot and the application have
     * the same paths and same home points
     */
    void newConnection(QString home_pos);

    /**
     * @brief connected
     * Sent we connect to the robot
     */
    void connected();
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
     * @brief sendCommand
     * @param cmd
     * @return
     * Called when we want to send a command
     */
    void sendCommand(const QString cmd);

    void writeTcpDataMP3Slot(const QString path, bool isLastMP3File);

    QVector<char> readSoundFile(QString path);

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
    int robotPort;
    int mapPort;
    int laserPort;
    int mp3Port;
    /// We need a QPointer because we can not initialize a QTimer in 1 thread then move the worker to another thread
    QPointer<QTimer> timer;
    int timeCounter;
};

#endif /// CMDROBOTWORKER_H
