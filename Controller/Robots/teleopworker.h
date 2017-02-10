#ifndef TELEOPWORKER_H
#define TELEOPWORKER_H

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>

class TeleopWorker : public QObject {
    Q_OBJECT
public:
    TeleopWorker(const QString _ipAddress, const int _port);
    ~TeleopWorker();

private slots:

    /**
     * @brief writeTcpDataSlot
     * @param cmd
     * Send the map to the robot
     */
    void writeTcpDataSlot(int cmd);

    /**
     * @brief connectSocket
     * Called to start the connection with the robot
     */
    void connectSocket();

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
};

#endif // TELEOPWORKER_H
