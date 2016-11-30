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
    void laserValues(float angle_min, float angle_max, float angle_increment, const QVector<float>& ranges, QString ipAddress);
    /// to add an entry for this robot in the obstacles map in order to draw the obstacles around it in real time
    void addNewRobotObstacles(QString ip);

private:
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    /// to store the values receive over the network
    QByteArray data;
};

#endif /// LOCALMAPWORKER_H
