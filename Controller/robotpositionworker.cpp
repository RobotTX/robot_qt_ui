#include "robotpositionworker.h"


RobotPositionWorker::RobotPositionWorker(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
}

RobotPositionWorker::~RobotPositionWorker(){
    stopThread();
}

void RobotPositionWorker::stopThread(){
    if(socket->isOpen())
        socket->close();
}

void RobotPositionWorker::connectSocket(){

    qDebug() << "(Robot pos thread" << ipAddress << ") Running";

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
    qDebug() << "(Robot pos thread" << ipAddress << ") We should be connected";

}

void RobotPositionWorker::readTcpDataSlot(){
    QString data = socket->readAll();
    QRegExp rx("[ ]");

    /// Data are received as a string separated by a space ("width height resolution originX originY")
    QStringList list = data.split(rx, QString::SkipEmptyParts);
    //qDebug() << "(Robot pos thread" << ipAddress << ") Position : " << list;
    emit valueChangedRobot(ipAddress, list.at(0).toDouble(), list.at(1).toDouble(), list.at(2).toDouble());
}

void RobotPositionWorker::disconnectedSlot(){
    qDebug() << "(Robot pos thread" << ipAddress << ") Disconnected";
}

void RobotPositionWorker::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Robot pos thread" << ipAddress << ") Error : " << error;
}

