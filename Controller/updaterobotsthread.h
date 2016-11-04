#ifndef UPDATEROBOTSTHREAD_H
#define UPDATEROBOTSTHREAD_H

#include <QTime>
#include <QThread>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>

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


private slots:
    void newConnectionSlot();
    void errorConnectionSlot(QAbstractSocket::SocketError error);
    void stopThread();

signals:
    void robotIsAlive(QString hostname, QString ip, QString mapId, QString ssid, int stage);

private:
    int port;
    QTcpServer* server;


};

#endif // UPDATEROBOTSTHREAD_H
