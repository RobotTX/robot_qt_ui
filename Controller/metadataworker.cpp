#include "metadataworker.h"

MetadataWorker::MetadataWorker(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;

}

MetadataWorker::~MetadataWorker(){
    stopThread();
}

void MetadataWorker::stopThread(){
    if(socket->isOpen())
        socket->close();
}

void MetadataWorker::connectSocket(){
    qDebug() << "(Robot Metadata thread" << ipAddress << ") Running";

    socket = QSharedPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal connected which trigger when we are connected to the host
    //connect(&(*socket), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlots(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));

    while(socket->state() != QAbstractSocket::ConnectedState){
        if(socket->state() == QAbstractSocket::UnconnectedState){
            socket->connectToHost(ipAddress, port);
            if(socket->waitForConnected(10000)){
                qDebug() << "(Robot Metadata thread" << ipAddress << ") Connected";
            } else {
                qDebug() << "(Robot Metadata thread" << ipAddress << ") Connecting error : " << socket->errorString() << " Trying again.";
            }
        }
    }
    qDebug() << "(Robot Metadata thread" << ipAddress << ") We should be connected";

}

void MetadataWorker::readTcpDataSlot(){
    QString data = socket->readAll();
    QRegExp rx("[ ]");

    /// Data are received as a string separated by a space ("width height resolution originX originY")
    QStringList list = data.split(rx, QString::SkipEmptyParts);
    //qDebug() << "(Robot Metadata thread" << ipAddress << ") Metadata : " << list;
    emit valueChangedMetadata(list.at(0).toInt(), list.at(1).toInt(),
                              list.at(2).toFloat(), list.at(3).toFloat(),
                              list.at(4).toFloat());
}

void MetadataWorker::disconnectedSlot(){
    qDebug() << "(Robot Metadata thread" << ipAddress << ") Disconnected";
}

void MetadataWorker::errorSlot(QAbstractSocket::SocketError socketError){
    qDebug() << "(Robot Metadata thread" << ipAddress << ") Error : " << socketError;
}
