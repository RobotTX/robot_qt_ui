#ifndef METADATAWORKER_H
#define METADATAWORKER_H

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>

/**
 * @brief The MetadataWorker class
 * The worker connect to the robot at the given ipAddress & port to receive the metadata
 * data of the map we want to display
 */
class MetadataWorker : public QObject {

    Q_OBJECT

public:
    /**
     * @brief MetadataWorker
     * @param ipAddress
     * @param port
     */
    MetadataWorker(const QString ipAddress, const int port);
    ~MetadataWorker();

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
     * @brief readTcpDataSlot
     * Read the data we receive
     */
    void readTcpDataSlot();

    /**
     * @brief stopWorker
     * Slot to stop the worker
     */
    void stopWorker();

    /**
     * @brief connectSocket
     * Called to start the connection with the robot
     */
    void connectSocket();

signals:
    /**
     * @brief valueChangedMetadata
     * @param width
     * @param height
     * @param resolution
     * @param originX
     * @param originY
     * Signal emitted when we have received the Metadata
     */
    void valueChangedMetadata(int width, int height, double resolution, double originX, double originY, double orientation);
    /**
     * @brief robotIsDead
     * notifies the robot controller that the connection has been lost
     */
    void robotIsDead();

private :
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;
};

#endif /// METADATAWORKER_H

