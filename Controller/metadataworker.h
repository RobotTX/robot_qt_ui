#ifndef METADATAWORKER_H
#define METADATAWORKER_H

#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>

/**
 * @brief The MetadataWorker class
 * The thread connect to the robot at the given ipAddress & port to receive the metadata
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
     * (Not connected) Slot called when there is an error
     */
    void errorConnectionSlot(QAbstractSocket::SocketError error);

    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();
    void readTcpDataSlot();
    void stopThread();
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
    void valueChangedMetadata(int width, int height, float resolution, float originX, float originY);

private :
    QPointer<QTcpSocket> socket;
    QString ipAddress;
    int port;

};


#endif // METADATAWORKER_H

