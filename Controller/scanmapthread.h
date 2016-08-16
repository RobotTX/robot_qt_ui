#ifndef SCANMAPTHREAD_H
#define SCANMAPTHREAD_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QSharedPointer>


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
    ScanMapThread(const QString ipAddress, const int port, const QString MapPath);

    /**
     * @brief run
     * Function called when we start a Thread
     */
    void run();


private slots:

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
    void newScanSaved(QString ipAddress);

private :
    QSharedPointer<QTcpSocket>socket;
    QString ipAddress;
    int port;
    /**
     * @brief data
     * The array on which we stack all the data we receive
     */
    QByteArray data;
    QString mapPath;

};


#endif // SCANMAPTHREAD_H
