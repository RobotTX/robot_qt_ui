#include "scanrobotthread.h"


ScanRobotThread::ScanRobotThread(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
}

void ScanRobotThread::run(){
    qDebug() << "(Robot pos thread" << ipAddress << ") Running";

    socket = QSharedPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socket), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socket, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));

    /// Connect to the host
    socket->connectToHost(ipAddress, port);

    int i = 1;
    while(!socket->waitForConnected(5000) && !isInterruptionRequested()){
        //qDebug() << "(Robot pos thread" << ipAddress << ") Attempt " << i << " :\nConnecting error : " << socket->errorString();
        socket->connectToHost(ipAddress, port);
        sleep(1);
        if(i++ > 100){
            qDebug() << "(Robot pos thread" << ipAddress << ") Too much connection attempts";
            socket -> close();
            exit();
            return;
        }
    }

    while (!this->isInterruptionRequested()) {
        if (!socket->waitForReadyRead()) {
            qDebug() << "(Robot pos thread" << ipAddress << ") Ready read error : " << socket->errorString();
            socket->close();
            exit();
            return;
        } else {
            QString data = socket->readAll();
            QRegExp rx("[ ]");
            QStringList list = data.split(rx, QString::SkipEmptyParts);

            /// Data are received as a string separated by a space ("width height resolution originX originY")
            emit valueChangedRobot(ipAddress, list.at(0).toDouble(), list.at(1).toDouble(), list.at(2).toDouble());
        }
    }
    socket->close();
    exit();
    return;
}

void ScanRobotThread::connectedSlot(){
    qDebug() << "(Robot pos thread" << ipAddress << ") Connected";
}

void ScanRobotThread::disconnectedSlot(){
    qDebug() << "(Robot pos thread" << ipAddress << ") Disconnected";
}

void ScanRobotThread::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Robot pos thread" << ipAddress << ") Error : " << error;
}
