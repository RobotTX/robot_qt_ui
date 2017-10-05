#ifndef LASERWORKER_H
#define LASERWORKER_H

#include <QObject>
#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>

class LaserWorker: public QObject
{
    Q_OBJECT
public:
    LaserWorker(const QString _ipAddress, const int _port);
    ~LaserWorker();

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

signals:
    /// used to draw the obstacles around the robot, the ranges represent the distance to the obstacles
    /// for each angle within the range, the range is [angle_min, angle_max]
    /// two angles differ by <angle_increment>
    void laserValues(float angle_min, float angle_max, float angle_increment, QVector<float> ranges);
    /// to notify the robot controller that the connection has been lost
    void robotIsDead();

private:
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    /// to store the values received over the network
    QByteArray data;
};

#endif /// LASERWORKER_H
