#include "scanmapworker.h"
#include <QDataStream>
#include <QFile>

ScanMapWorker::ScanMapWorker(const QString newipAddress, const int newPort, const QString _mapPath){
    ipAddress = newipAddress;
    port = newPort;
    data = QByteArray();
    mapPath = _mapPath;
}

ScanMapWorker::~ScanMapWorker(){
    stopThread();
}

void ScanMapWorker::stopThread(){
    if(socket && socket->isOpen())
        socket->close();
}


void ScanMapWorker::connectSocket(){
    qDebug() << "(Map Thread) Trying to connect to" << ipAddress << "at port" << port;

    socket = QSharedPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal connected which trigger when we are connected to the host
    //connect(&(*socket), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));


    while(socket->state() != QAbstractSocket::ConnectedState){
        if(socket->state() == QAbstractSocket::UnconnectedState){
            socket->connectToHost(ipAddress, port);
            if(socket->waitForConnected(10000)){
                qDebug() << "(Robot pos thread" << ipAddress << ") Connected";
            } else {
                qDebug() << "(Robot pos thread" << ipAddress << ") Connecting error : " << socket->errorString() << " Trying again.";
            }
        }
    }
    qDebug() << "(Map Thread) We should be connected";
}

void ScanMapWorker::readTcpDataSlot(){
    qDebug() << "(Map) Received data";
    data.append(socket->readAll());

    /// The TCP protocol sending blocks of data, a map is defined by a random number
    /// of blocks, so we wait till the last byte of a block is -2, meaning we have received
    /// a complete map
    if(data.size() >= 5 && ((int) data.at(data.size()-5) == 0  && (int) data.at(data.size()-4) == 0  && (int) data.at(data.size()-3) == 0
            && (int) data.at(data.size()-2) == 0  && (int) data.at(data.size()-1) == -2)){

        qDebug() << "(Map) Map of" << data.size() << "bytes received";
        /// Remove the end bytes 0 0 0 0 -2 as we no longer need it
        data.remove(data.size()-5,5);
        /// Emit the signal valueChangedMap, meaning that we finished to receive a whole map
        /// and we can display it
        emit valueChangedMap(data);
        /// Clear the Vector that contain the map, once it has been treated
        data.clear();
    } else if(data.at(data.size()-1) == -3){
        ///We are receiving the real map
        data.remove(data.size()-1,1);
        qDebug() << "(Map) Real map of" << data.size() << "bytes received <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
        QFile file(mapPath);
        file.resize(0);
        file.open(QIODevice::WriteOnly);
        file.write(data);
        file.close();
        data.clear();
        emit newScanSaved(ipAddress);
    }
}

void ScanMapWorker::disconnectedSlot(){
    qDebug() << "(Map) Disconnected";
}

void ScanMapWorker::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Map) Error : " << error;
}
