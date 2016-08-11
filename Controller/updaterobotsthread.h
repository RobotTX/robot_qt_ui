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
    void delay(const int ms) const;


private slots:
    void newConnectionSlot();
    void disconnectedSlot();
    void readTcpDataSlot();
    void errorConnectionSlot(QAbstractSocket::SocketError error);

signals:
    void robotIsAlive(QString hostname, QString ip, QString mapId, QString ssid, int stage);

private:
    int port;
    QTcpServer* server;
    QTcpSocket* socket;


};

#endif // UPDATEROBOTSTHREAD_H
