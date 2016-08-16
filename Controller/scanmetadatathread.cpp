#include "scanmetadatathread.h"

ScanMetadataThread::ScanMetadataThread(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
}

void ScanMetadataThread::run(){
    qDebug() << "Metadata Thread";

    socket = QSharedPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socket), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlots(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));
    /// Connect to the host
    socket->connectToHost(ipAddress, port);

    int i = 1;
    while(!socket->waitForConnected(5000) && !isInterruptionRequested()){
        //qDebug() << "(Metadata) Attempt " << i << " :\nConnecting error : " << socket->errorString();
        socket->connectToHost(ipAddress, port);
        sleep(1);
        if(i++ > 100){
            qDebug() << "(Metadata) Too much connection attempts";
            socket -> close();
            exit();
            return;
        }
    }

    while (!this->isInterruptionRequested()) {
        if (!socket->waitForReadyRead()) {
            qDebug() << "(Metadata) Ready read error : " << socket->errorString();
            socket -> close();
            exit();
            return;
        } else {
            QString data = socket->readAll();
            QRegExp rx("[ ]");
            qDebug() << "(Robot) Metadata" << data;

            /// Data are received as a string separated by a space ("width height resolution originX originY")
            QStringList list = data.split(rx, QString::SkipEmptyParts);
            emit valueChangedMetadata(list.at(0).toInt(), list.at(1).toInt(),
                                      list.at(2).toFloat(), list.at(3).toFloat(),
                                      list.at(4).toFloat());
        }
    }
    socket->close();
    exit();
    return;
}

void ScanMetadataThread::connectedSlot(){
    qDebug() << "(Metadata) Connected";
}

void ScanMetadataThread::disconnectedSlot(){
    qDebug() << "(Metadata) Disconnected";
}

void ScanMetadataThread::errorSlot(QAbstractSocket::SocketError socketError){
    qDebug() << "(Metadata) Error : " << socketError;
}
