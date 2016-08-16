#include "sendnewmapthread.h"

SendNewMapThread::SendNewMapThread(const QString _ipAddress, const int _port){
    ipAddress = _ipAddress;
    port = _port;
    connected = false;
}

void SendNewMapThread::run(){
    qDebug() << "(New Map) Command Thread launched";

    socket = QSharedPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socket), SIGNAL(connected()), this, SLOT(connectedSlot()));
    //connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));

    /// Connect to the host
    socket->connectToHost(ipAddress, port);

    int i = 1;
    while(!socket->waitForConnected(5000) && !isInterruptionRequested()){
        socket->connectToHost(ipAddress, port);
        sleep(1);
        if(i++ > 100){
            qDebug() << "(New Map) Too much connection attempts";
            socket -> close();
            exit();
            return;
        }
    }

    while(!isInterruptionRequested()){
        if(!socket->isOpen()){
            exit();
            return;
        }
        if (socket->waitForReadyRead(1000)) {
            QString dataStr = socket->readAll();
            qDebug() << "(New Map) data received :" << dataStr;
            emit doneSendingNewMapSignal();
        }
    }
    socket->close();
    exit();
    return;
}

void SendNewMapThread::writeTcpDataSlot(QByteArray cmd){
    qDebug() << "(New Map) Sending the new map to" << ipAddress << "at port " << port;

    if(!connected)
        return;

    cmd.push_back((int8_t) -2);

    int nbDataSend = socket->write(cmd);

    socket->waitForBytesWritten();

    if(nbDataSend == -1){
        qDebug() << "(New Map) An error occured while sending data";
    } else {
        qDebug() << "(New Map) " << nbDataSend << "bytes sent";
    }
}

void SendNewMapThread::connectedSlot(){
    qDebug() << "(New Map) Connected";
    connected = true;
}

void SendNewMapThread::disconnectedSlot(){
    qDebug() << "(New Map) Disconnected at ip" << ipAddress;
    connected = false;
    socket->close();
    exit();
    return;
}
