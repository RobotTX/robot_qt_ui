#include "scanmapthread.h"
ScanMapThread::ScanMapThread(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
    data = QByteArray();
}

void ScanMapThread::run(){
    qDebug() << "Map Thread";

    socketMap = std::shared_ptr<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    //connect(&(*socketMap), SIGNAL(readyRead()), SLOT(readTcpDataSlot()) );
    /// Connect the signal hostFound which trigger when we find the host
    //connect( socketMap, SIGNAL(hostFound()), SLOT(hostFoundSlot()) );
    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socketMap), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socketMap, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );   
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socketMap), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));

    /// Connect to the host
    socketMap->connectToHost(ipAddress, port);

    int i = 1;
    while(!socketMap->waitForConnected(5000) && !isInterruptionRequested()){
        //qDebug() << "(Map) Attempt " << i << " :\nConnecting error : " << socketMap->errorString();
        socketMap->connectToHost(ipAddress, port);
        sleep(1);
        if(i++ > 100){
            qDebug() << "(Map) Too much connection attempts";
            socketMap -> close();
            exit();
            return;
        }
    }

    /// Throw an error if bytes are available but we can't read them
    /*while (socketMap->bytesAvailable() < (int)sizeof(quint16)) {
        if (!socketMap->waitForReadyRead()) {
            qDebug() << "(Map) Ready read error : " << socketMap->errorString();
            socketMap -> close();
            exit();
            return;
        }
    }*/
    while(!this->isInterruptionRequested()){
        if (!socketMap->waitForReadyRead()) {
            qDebug() << "(Map) Ready read error : " << socketMap->errorString();
            socketMap -> close();
            exit();
            return;
        }
        data.append(socketMap->readAll());

        /// The TCP protocol sending blocks of data, a map is defined by a random number
        /// of blocks, so we wait till the last byte of a block is -2, meaning we have received
        /// a complete map
        if((int) data.at(data.size()-1) == -2 ){

            qDebug() << "(Robot) map " << data.at(0) << data.at(1) << data.at(2) << data.at(3) << data.at(4);
            /// Remove the byte -2 as we no longer need it
            data.remove(data.size()-1,1);
            /// Emit the signal valueChangedMap, meaning that we finished to receive a whole map
            /// and we can display it
            emit valueChangedMap(data);
            /// Clear the Vector that contain the map, once it has been treated
            data.clear();
        }
    }
}

void ScanMapThread::readTcpDataSlot(){
    data.append(socketMap->readAll());

    /// The TCP protocol sending blocks of data, a map is defined by a random number
    /// of blocks, so we wait till the last byte of a block is -2, meaning we have received
    /// a complete map
    if((int) data.at(data.size()-1) == -2 ){
        /// Remove the byte -2 as we no longer need it
        data.remove(data.size()-1,1);
        /// Emit the signal valueChangedMap, meaning that we finished to receive a whole map
        /// and we can display it
        emit valueChangedMap(data);
        /// Clear the Vector that contain the map, once it has been treated
        data.clear();
    }
}

void ScanMapThread::hostFoundSlot(){
    qDebug() << "(Map) Host found";
}

void ScanMapThread::connectedSlot(){
    qDebug() << "(Map) Connected";
}

void ScanMapThread::disconnectedSlot(){
    qDebug() << "(Map) Disconnected";
}

void ScanMapThread::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Map) Error : " << error;
}
