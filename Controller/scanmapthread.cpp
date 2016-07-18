#include "scanmapthread.h"
#include <QDataStream>

ScanMapThread::ScanMapThread(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
    data = QByteArray();
}

void ScanMapThread::run(){
    qDebug() << "Map Thread";

    socketMap = std::shared_ptr<QTcpSocket>(new QTcpSocket());

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

    while(!this->isInterruptionRequested()){
        if (!socketMap->waitForReadyRead()) {
            qDebug() << "(Map) Ready read error : " << socketMap->errorString();
            socketMap -> close();
            exit();
            return;
        } else {
            data.append(socketMap->readAll());

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
            }
        }
    }
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
