#ifndef SCANMETADATATHREAD_H
#define SCANMETADATATHREAD_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>

/**
 * @brief The ScanMetadataThread class
 * The thread connect to the robot at the given ipAddress & port to receive the metadata
 * data of the map we want to display
 */
class ScanMetadataThread : public QThread {
    Q_OBJECT
public:
    /**
     * @brief ScanMetadataThread
     * @param ipAddress
     * @param port
     */
    ScanMetadataThread(QString ipAddress, int port);
    ~ScanMetadataThread();

    /**
     * @brief run
     * Function called when we start a Thread
     */
    void run();

private slots:
    /**
     * @brief readTcpData
     * Read the data we receive
     */
    void readTcpData();

    /**
     * @brief hostFoundSlot
     * Slot called when an host is found
     */
    void hostFoundSlot();

    /**
     * @brief connectedSlot
     * Slot called when we are connected to the host
     */
    void connectedSlot();

    /**
     * @brief errorSlot
     * @param error
     * (Not connected) Slot called when there is an error
     */
    void errorSlot(QAbstractSocket::SocketError error);

    /**
     * @brief disconnectedSlot
     * Slot called when we are disconnected from the host
     */
    void disconnectedSlot();

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
    QTcpSocket* socketMetadata;
    QString ipAddress;
    int port;

};


#endif // SCANMETADATATHREAD_H

