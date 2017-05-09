#ifndef SCANMAPWORKER_H
#define SCANMAPWORKER_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>


/**
 * @brief The ScanMapWorker class
 * The thread connect to the robot at the given ipAddress & port to receive the map the robot
 * is scanning
 */
class ScanMapWorker : public QObject {
    Q_OBJECT
public:
    /**
     * @brief ScanMapWorker::MapThread
     * @param newipAddress
     * @param newPort
     */
    ScanMapWorker(const QString ipAddress, const int port);
    ~ScanMapWorker();

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

signals:
    /**
     * @brief valueChangedMap
     * Signal emmited when we finished to receive a whole map
     * and we can display it
     */
    void valueChangedMap(QByteArray, int, QString, QString, QString, QString, QString, int, int);
    /**
     * @brief robotIsDead
     * to notify the robot controller that the connection cannot be established
     */
    void robotIsDead();

private :
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    /**
     * @brief data
     * The array in which we stack all the data we receive
     */
    QByteArray data;
};


#endif /// SCANMAPWORKER_H
