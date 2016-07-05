#ifndef SCANMAPTHREAD_H
#define SCANMAPTHREAD_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <memory>
/**
 * @brief The ScanMapThread class
 * The thread connect to the robot at the given ipAddress & port to receive the map the robot
 * is scanning
 */
class ScanMapThread : public QThread {
    Q_OBJECT
public:
    /**
     * @brief ScanMapThread::MapThread
     * @param newipAddress
     * @param newPort
     */
    ScanMapThread(const QString ipAddress, const int port);

    /**
     * @brief run
     * Function called when we start a Thread
     */
    void run();


private slots:
    /**
     * @brief readTcpDataSlot
     * Read the data we receive
     */
    void readTcpDataSlot();

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
     * @brief valueChangedMap
     * Signal emmited when we finished to receive a whole map
     * and we can display it
     */
    void valueChangedMap(QByteArray);

private :
    std::shared_ptr<QTcpSocket>socketMap;
    QString ipAddress;
    int port;
    /**
     * @brief data
     * The array on which we stack all the data we receive
     */
    QByteArray data;

};


#endif // SCANMAPTHREAD_H
