#ifndef SCANMAPWORKER_H
#define SCANMAPWORKER_H

#include <QThread>
#include <QString>
#include <QtNetwork/QTcpSocket>
#include <QPointer>


/**
 * @brief The ScanMapWorker class
 * The thread connect to the robot at the given ipAddress & port to receive the map the robot
 * is scanning
 */
class ScanMapWorker : public QObject {
    Q_OBJECT
public:
    /**
     * @brief ScanMapWorker::MapThread
     * @param newipAddress
     * @param newPort
     */
    ScanMapWorker(const QString ipAddress, const int port, const QString MapPath);
    ~ScanMapWorker();


private slots:

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
    void readTcpDataSlot();
    void stopThread();
    void connectSocket();

signals:
    /**
     * @brief valueChangedMap
     * Signal emmited when we finished to receive a whole map
     * and we can display it
     */
    void valueChangedMap(QByteArray);
    void newScanSaved(QString ipAddress);

private :
    QPointer<QTcpSocket>socket;
    QString ipAddress;
    int port;
    /**
     * @brief data
     * The array on which we stack all the data we receive
     */
    QByteArray data;
    QString mapPath;

};


#endif // SCANMAPWORKER_H
