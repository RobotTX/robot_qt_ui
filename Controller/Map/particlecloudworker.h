#ifndef PARTICLECLOUDWORKER_H
#define PARTICLECLOUDWORKER_H

#include <QObject>
#include <QPointer>
#include <QtNetwork/QTcpSocket>
#include <QThread>

class ParticleCloudWorker: public QObject {

    Q_OBJECT

public:
    ParticleCloudWorker(const QString _ipAddress, const int _port);
    ~ParticleCloudWorker();

private slots:
    /**
     * @brief errorConnectionSlot
     * @param error
     * Called when an error occurs with the socket
     * When we try to connectToHost, if the socket on the robot is not open yet (or other problem),
     * we'll try to connect again in this slot
     */
    void errorConnectionSlot(QAbstractSocket::SocketError error);

    /**
     * @brief stopWorker
     * Slot to stop the worker
     */
    void stopWorker();
    /**
     * @brief readTcpDataSlot
     * Read the data we receive
     */
    void readTcpDataSlot();

    /**
     * @brief connectSocket
     * Called to start the connection with the robot
     */
    void connectSocket();

private:
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    /// to store the values received over the network
    QByteArray data;
};

#endif /// PARTICLECLOUDWORKER_H
