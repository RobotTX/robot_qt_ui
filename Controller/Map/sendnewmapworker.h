#ifndef SENDNEWMAPWORKER_H
#define SENDNEWMAPWORKER_H

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>
#include <QImage>

class SendNewMapWorker : public QObject {
    Q_OBJECT
public:
    SendNewMapWorker(const QString _ipAddress, const int _port);
    ~SendNewMapWorker();

signals:
    void doneSendingNewMapSignal();

private slots:

    /**
     * @brief readTcpDataSlot
     * Read the data we receive
     */
    void readTcpDataSlot();

    /**
     * @brief writeTcpDataSlot
     * @param mapId
     * @param date
     * @param metadata
     * @param map
     * Send the map to the robot
     */
    void writeTcpDataSlot(QString mapId, QString date, QString metadata, QImage map);

    /**
     * @brief connectSocket
     * Called to start the connection with the robot
     */
    void connectSocket();

    /**
     * @brief stopWorker
     * Slot to stop the worker
     */
    void stopWorker();

    /**
     * @brief errorConnectionSlot
     * @param error
     * Called when an error occurs with the socket
     * When we try to connectToHost, if the socket on the robot is not open yet (or other problem),
     * we'll try to connect again in this slot
     */
    void errorConnectionSlot(QAbstractSocket::SocketError error);

private :
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
};

#endif // SENDNEWMAPWORKER_H
