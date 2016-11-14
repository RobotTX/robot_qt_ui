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

    void startServer();


private slots:
    void newConnectionSlot();
    void errorConnectionSlot(QAbstractSocket::SocketError error);
    void stopThread();

signals:
    void robotIsAlive(QString hostname, QString ip, QString mapId, QString ssid, int stage);

private:
    int port;
    //QTcpServer* server;
};

#endif /// ROBOT_SERVER_WORKER_H
