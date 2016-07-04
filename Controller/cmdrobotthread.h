#ifndef CMDROBOTTHREAD_H
#define CMDROBOTTHREAD_H

class QtNetwork;
class QTcpSocket;

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <memory>

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
    CmdRobotThread(const QString ipAddress, const int port, const QString _robotName);

    /**
     * @brief run
     * Function called when we start a Thread
     */
    void run();

    /**
     * @brief sendCommand
     * @param cmd
     * @return
     * Called when we want to send a command
     */
    bool sendCommand(const QString cmd);
    QString waitAnswer();
    void resetCommandAnswer(){ commandAnswer = ""; }


private slots:
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

    /**
     * @brief readTcpData
     * Read the data we receive
     */
    void readTcpData();

private :
    std::shared_ptr<QTcpSocket> socketCmd;
    QString ipAddress;
    int port;
    QString robotName;
    bool connected;
    QString commandAnswer;
};

#endif // CMDROBOTTHREAD_H
