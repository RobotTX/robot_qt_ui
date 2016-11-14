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
     * @brief errorSlot
     * @param error
     * (Not connected) Slot called when there is an error
     */
    void errorSlot(QAbstractSocket::SocketError error);

    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();
    void readTcpDataSlot();
    void stopThread();
    void connectSocket();


signals:
    /**
     * @brief valueChangedRobot
     * @param posX
     * @param posY
     * @param ori
     * Signal emitted when we have received the robot position & orientation
     */
    void valueChangedRobot(QString ipAddress, float posX, float posY, float ori);

private :
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;

};


#endif // ROBOTPOSITIONWORKER_H
