#ifndef ROBOTPOSITIONWORKER_H
#define ROBOTPOSITIONWORKER_H

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>

/**
 * @brief The RobotThread class
 * The threadScan connect to the robot at the given ipAddress & port to receive
 * the robot position & orientation
 */
class RobotPositionWorker : public QObject {
    Q_OBJECT

public:
    /**
     * @brief RobotPositionWorker
     * @param ipAddress
     * @param port
     */
    RobotPositionWorker(const QString ipAddress, const int port);
    ~RobotPositionWorker();

private slots:

    /**
     * @brief readTcpDataSlot
     * Read the data we receive
     */
    void readTcpDataSlot();

    /**
     * @brief stopWorker
     * Slot to stop the worker
     */
    void stopWorker();

    /**
     * @brief connectSocket
     * Called to start the connection with the robot
     */
    void connectSocket();

    /**
     * @brief errorConnectionSlot
     * @param error
     * Called when an error occurs with the socket
     * When we try to connectToHost, if the socket on the robot is not open yet (or other problem),
     * we'll try to connect again in this slot
     */
    void errorConnectionSlot(QAbstractSocket::SocketError error);

signals:
    /**
     * @brief valueChangedRobot
     * @param posX
     * @param posY
     * @param ori
     * Signal emitted when we have received the robot position & orientation
     */
    void valueChangedRobot(double posX, double posY, double ori);
    /**
     * @brief robotIsDead
     * If we don't receive the position of the robot we notify the rest of the application that the robot is dead
     */
    void robotIsDead();

private :
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
};


#endif /// ROBOTPOSITIONWORKER_H
