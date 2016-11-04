#ifndef SENDNEWMAPWORKER_H
#define SENDNEWMAPWORKER_H

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QSharedPointer>

class SendNewMapWorker : public QObject {
    Q_OBJECT
public:
    SendNewMapWorker(const QString _ipAddress, const int _port);
    ~SendNewMapWorker();

signals:
    void doneSendingNewMapSignal();

private slots:

    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();

    void readTcpDataSlot();
    void writeTcpDataSlot(QByteArray cmd);
    void connectSocket();
    void stopThread();

private :
    QSharedPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
    bool connected;
};

#endif // SENDNEWMAPWORKER_H
