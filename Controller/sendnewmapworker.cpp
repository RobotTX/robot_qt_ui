#include "sendnewmapworker.h"

SendNewMapWorker::SendNewMapWorker(const QString _ipAddress, const int _port){
    ipAddress = _ipAddress;
    port = _port;
}

SendNewMapWorker::~SendNewMapWorker(){
    stopThread();
}

void SendNewMapWorker::stopThread(){
    if(socket->isOpen())
        socket->close();
}

void SendNewMapWorker::connectSocket(){
    qDebug() << "(Robot new map thread" << ipAddress << ") Running";

    socket = QSharedPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal connected which trigger when we are connected to the host
    //connect(&(*socket), SIGNAL(connected()), this, SLOT(connectedSlot()));
    //connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));

    /// Connect to the host
    while(socket->state() != QAbstractSocket::ConnectedState){
        if(socket->state() == QAbstractSocket::UnconnectedState){
            socket->connectToHost(ipAddress, port);
            if(socket->waitForConnected(10000)){
                qDebug() << "(Robot new map thread" << ipAddress << ") Connected";
            } else {
                qDebug() << "(Robot new map thread" << ipAddress << ") Connecting error : " << socket->errorString() << " Trying again.";
            }
        }
    }
}

void SendNewMapWorker::readTcpDataSlot(){
    QString dataStr = socket->readAll();
    qDebug() << "(New Map) data received :" << dataStr;
    emit doneSendingNewMapSignal();
}

void SendNewMapWorker::writeTcpDataSlot(QByteArray cmd){
    qDebug() << "(New Map) Sending the new map to" << ipAddress << "at port " << port;

    if(!connected)
        return;

    cmd.push_back((int8_t) -2);

    int nbDataSend = socket->write(cmd);

    socket->waitForBytesWritten();

    if(nbDataSend == -1){
        qDebug() << "(New Map) An error occured while sending data";
    } else {
        qDebug() << "(New Map) " << nbDataSend << "bytes sent out of" << cmd.size();
    }
}

void SendNewMapWorker::disconnectedSlot(){
    qDebug() << "(New Map) Disconnected at ip" << ipAddress;
}
