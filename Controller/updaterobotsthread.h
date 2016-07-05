#ifndef UPDATEROBOTSTHREAD_H
#define UPDATEROBOTSTHREAD_H

#include <QThread>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <memory>
#include <QTime>

class UpdateRobotsThread : public QThread {
    Q_OBJECT
public:
    UpdateRobotsThread(const int port);
    ~UpdateRobotsThread();

    /**
     * @brief run
     * Function called when we start a Thread
     */
    void run();
    void delay(const int ms) const;

public:
    struct RobotStruct {
        QString hostname;
        QString ip;
        QTime lastTimeConnected;
    };

private slots:
    void newConnectionSlot();
    void disconnectedSlot();
    void readTcpDataSlot();
    void errorConnectionSlot(QAbstractSocket::SocketError error);

private:
    int port;
    QTcpServer* server;
    QTcpSocket* socket;
    QVector<RobotStruct> robotsVector;


};

#endif // UPDATEROBOTSTHREAD_H
