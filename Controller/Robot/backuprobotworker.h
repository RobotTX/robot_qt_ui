#ifndef BACKUPROBOTWORKER_H
#define BACKUPROBOTWORKER_H

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>
#include <QTimer>

class BackupRobotWorker : public QObject {

    Q_OBJECT

public:
    explicit BackupRobotWorker(const QString _ipAddress, const int _port);
    ~BackupRobotWorker();

private slots:

    /**
     * @brief connectSocket
     * Called to start the connection with the robot
     */
    void connectSocket();

    /**
     * @brief stopWorker
     * To stop the worker
     */
    void stopWorker(void);

    /**
     * @brief connectedSlot
     * Slot called when we are connected to the host
     */
    void connectedSlot();

    /**
     * @brief errorConnectionSlot
     * @param error
     * Called when an error occurs with the socket
     * When we try to connectToHost, if the socket on the robot is not open yet (or other problem),
     * we'll try to connect again in this slot
     */
    void errorConnectionSlot(QAbstractSocket::SocketError error);
    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();

    /**
     * @brief readTcpDataSlot
     * read the feedback from robot side (done, fail ...)
     */
    void readTcpDataSlot(void);

    /**
     * @brief callForReboot
     * sends a message to the backup system to reboot the robot (restart roscore,
     * gobot_move, gobot_software and all packages necessary)
     */
    void callForReboot(void);

signals:
    void backupSystemIsDown(QString ip);

private:
    QString ipAddress;
    int port;
    QPointer<QTcpSocket> socket;
};

#endif /// BACKUPROBOTWORKER_H
