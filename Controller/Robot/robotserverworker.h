#ifndef ROBOT_SERVER_WORKER_H
#define ROBOT_SERVER_WORKER_H

#include <QTime>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QDebug>

class RobotServerWorker : public QTcpServer {

    Q_OBJECT

public:

    explicit RobotServerWorker(const int port, QObject* parent = 0);

    /**
     * @brief startServer
     * start listening on port <port>
     */
    void startServer();

private slots:
    /**
     * @brief newConnectionSlot
     * to add a new robot to the application
     */
    void newConnectionSlot();
    void errorConnectionSlot(QAbstractSocket::SocketError error);
    void stopWorker();

signals:
    /**
     * @brief robotIsAlive
     * @param hostname
     * @param ip
     * @param stage
     * @param battery
     * notifies the robots controller that a new robot has connected
     */
    void robotIsAlive(QString hostname, QString ip, int stage, int battery, bool charging, int dockStatus);

private:
    int port;
};

#endif /// ROBOT_SERVER_WORKER_H
