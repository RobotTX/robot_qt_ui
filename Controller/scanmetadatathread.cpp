#include "scanmetadatathread.h"

ScanMetadataThread::ScanMetadataThread(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
    ok = true;
}

void ScanMetadataThread::run(){
    qDebug() << "Metadata Thread";

    socketMetadata = std::shared_ptr<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    //connect(&(*socketMetadata), SIGNAL(readyRead()), SLOT(readTcpData()) );
    /// Connect the signal hostFound which trigger when we find the host
    //connect( socketMetadata, SIGNAL(hostFound()), SLOT(hostFoundSlot()) );
    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socketMetadata), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socketMetadata, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlots(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socketMetadata), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));
    /// Connect to the host
    socketMetadata->connectToHost(ipAddress, port);

    int i = 1;
    while(!socketMetadata->waitForConnected(5000)){
        //qDebug() << "(Metadata) Attempt " << i << " :\nConnecting error : " << socketMetadata->errorString();
        socketMetadata->connectToHost(ipAddress, port);
        sleep(1);
        if(i++ > 100){
            qDebug() << "(Metadata) Too much connection attempts";
            socketMetadata -> close();
            exit();
            return;
        }
    }

    /// Throw an error if bytes are available but we can't read them
    /*while (socketMetadata->bytesAvailable() < (int)sizeof(quint16)) {
        if (!socketMetadata->waitForReadyRead()) {
            qDebug() << "(Metadata) Ready read error : " << socketMetadata->errorString();
            socketMetadata -> close();
            exit();
            return;
        }
    }*/
    while (ok) {
        if (!socketMetadata->waitForReadyRead()) {
            qDebug() << "(Metadata) Ready read error : " << socketMetadata->errorString();
            socketMetadata -> close();
            exit();
            return;
        }
        QString data = socketMetadata->readAll();
        QRegExp rx("[ ]");
        qDebug() << "(Robot) Metadata" << data;

        /// Data are received as a string separated by a space ("width height resolution originX originY")
        QStringList list = data.split(rx, QString::SkipEmptyParts);
        emit valueChangedMetadata(list.at(0).toInt(), list.at(1).toInt(),
                                  list.at(2).toFloat(), list.at(3).toFloat(),
                                  list.at(4).toFloat());
    }
}

void ScanMetadataThread::readTcpData(){
    QString data = socketMetadata->readAll();
    QRegExp rx("[ ]");

    /// Data are received as a string separated by a space ("width height resolution originX originY")
    QStringList list = data.split(rx, QString::SkipEmptyParts);
    emit valueChangedMetadata(list.at(0).toInt(), list.at(1).toInt(),
                              list.at(2).toFloat(), list.at(3).toFloat(),
                              list.at(4).toFloat());
}

void ScanMetadataThread::hostFoundSlot(){
    qDebug() << "(Metadata) Host found";
}

void ScanMetadataThread::connectedSlot(){
    qDebug() << "(Metadata) Connected";
}

void ScanMetadataThread::disconnectedSlot(){
    qDebug() << "(Metadata) Disconnected";
    ok = false;
}

void ScanMetadataThread::errorSlot(QAbstractSocket::SocketError socketError){
    qDebug() << "(Metadata) Error : " << socketError;
}
