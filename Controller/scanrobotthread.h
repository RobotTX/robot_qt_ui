#ifndef SCANROBOTTHREAD_H
#define SCANROBOTTHREAD_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>

/**
 * @brief The RobotThread class
 * The thread connect to the robot at the given ipAddress & port to receive
 * the robot position & orientation
 */
class ScanRobotThread : public QThread {
    Q_OBJECT
public:
    /**
     * @brief ScanRobotThread
     * @param ipAddress
     * @param port
     */
    ScanRobotThread(QString ipAddress, int port);
    ~ScanRobotThread();

    /**
     * @brief run
     * Function called when we start a Thread
     */
    void run();

private slots:
    /**
     * @brief readTcpData
     * Read the data we receive
     */
    void readTcpData();

    /**
     * @brief hostFoundSlot
     * Slot called when an host is found
     */
    void hostFoundSlot();

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

    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();

signals:
    /**
     * @brief valueChangedRobot
     * @param posX
     * @param posY
     * @param ori
     * Signal emitted when we have received the robot position & orientation
     */
    void valueChangedRobot(float posX, float posY, float ori);

private :
    QTcpSocket* socketRobot;
    QString ipAddress;
    int port;

};


#endif // SCANROBOTTHREAD_H
