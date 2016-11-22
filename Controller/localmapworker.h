#ifndef LOCALMAPWORKER_H
#define LOCALMAPWORKER_H

#include <QObject>
#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>

class LocalMapWorker: public QObject
{
    Q_OBJECT
public:
    LocalMapWorker(const QString _ipAddress, const int _port);
    ~LocalMapWorker();

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
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();
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
    QByteArray data;
};

#endif /// LOCALMAPWORKER_H
