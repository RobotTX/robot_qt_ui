#ifndef SENDNEWMAPTHREAD_H
#define SENDNEWMAPTHREAD_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QSharedPointer>

class SendNewMapThread : public QThread {
    Q_OBJECT
public:
    SendNewMapThread(const QString _ipAddress, const int _port);
    void run();
    bool isConnected() const {return connected;}

signals:
    void doneSendingNewMapSignal();

private slots:
    /**
     * @brief connectedSlot
     * Slot called when we are connected to the host
     */
    void connectedSlot();

    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();

    void writeTcpDataSlot(QByteArray cmd);

private :
    QSharedPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    bool connected;
};

#endif // SENDNEWMAPTHREAD_H
