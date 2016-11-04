#include "cmdrobotworker.h"
#include <QTime>
#include <QCoreApplication>


CmdRobotWorker::CmdRobotWorker(const QString _ipAddress, const int cmdPort, const int _metadataPort, const int _robotPort, const int _mapPort, const QString _robotName){
    ipAddress = _ipAddress;
    port = cmdPort;
    robotName = _robotName;
    missedPing = MISSED_PING_TIMER;
    metadataPort = _metadataPort;
    robotPort = _robotPort;
    mapPort = _mapPort;
}

CmdRobotWorker::~CmdRobotWorker(){
    stopCmdRobotWorkerSlot();
}

void CmdRobotWorker::stopCmdRobotWorkerSlot(){
    if(socket && socket->isOpen())
        socket->close();
    //exit();
}

void CmdRobotWorker::connectSocket(){

    qDebug() << "(Robot" << robotName << ") Command Thread launched";

    socket = QSharedPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));
    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socket), SIGNAL(connected()), this, SLOT(connectedSlot()));
    //connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));

    /// Connect to the host
    socket->connectToHost(ipAddress, port);

    QString portStr = "h \"" + QString::number(metadataPort) + "\" \"" + QString::number(robotPort) + "\" \"" + QString::number(mapPort) + "\" } ";
    qDebug() << "(Robot" << robotName << ") Sending ports : " << portStr;
    bool tmpBool = 0;
    while(!tmpBool){
        socket->write(portStr.toUtf8());
        if(socket->waitForBytesWritten(100)){
            qDebug() << "(Robot" << robotName << ") Ports sent";
            tmpBool = 1;
            emit portSent();
        } else {
            qDebug() << "(Robot" << robotName << ") Ports could not be sent";
        };
    }


/*
    qDebug() << "(Robot" << robotName << ") Done";
    while(!isInterruptionRequested()){
        if(!socket->isOpen()){
            exit();
            return;
        }
        if(missedPing <= 0){
            qDebug() << "missedPing robotIsDead";
            emit robotIsDead(robotName, ipAddress);
            socket -> close();
            exit();
            return;
        }
        delay(500);
        missedPing--;
        if(missedPing <= 4)
            qDebug() << "Received no ping during the last :" << (float) ((MISSED_PING_TIMER - missedPing)/2) << " seconds.";
    }
    socket->close();
    exit();
    return;*/
    //exec();
}

void CmdRobotWorker::sendCommand(const QString cmd){
    qDebug() << "(Robot" << robotName << ") Command to send :" << cmd << "to" << ipAddress << "at port " << port;
    int nbDataSend = socket->write(QString(cmd + " } ").toUtf8());

    socket->waitForBytesWritten(100);

    if(nbDataSend == -1){
        qDebug() << "(Robot" << robotName << ") An error occured while sending data";
    } else {
        qDebug() << "(Robot" << robotName << ") " << nbDataSend << " bytes sent";

        /*if(QString(cmd.at(0)).compare("e")){
            socket->waitForReadyRead(100);
        } else {
            qDebug() << "No wait";
            delay(2000);
        }*/
    }
    qDebug() << "(Robot" << robotName << ") Command sent successfully";
}

void CmdRobotWorker::readTcpDataSlot(){
    QString commandAnswer = socket->readAll();
    qDebug() << "(Robot" << robotName << ") readTcpDataSlot :" << commandAnswer;
    emit cmdAnswer(commandAnswer);
    missedPing = MISSED_PING_TIMER;
}

void CmdRobotWorker::connectedSlot(){
    qDebug() << "(Robot" << robotName << ") Connected";
}

void CmdRobotWorker::disconnectedSlot(){
    qDebug() << "(Robot" << robotName << ") Disconnected at ip" << ipAddress;
    if(robotName.compare("") != 0){
        qDebug() << "(Robot" << robotName << ") Emitting robotIsDead";
        emit robotIsDead(robotName, ipAddress);
        if(socket->isOpen())
            socket->close();
    }
}

void CmdRobotWorker::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Robot" << robotName << ") Catched a connection error : " << error;
}

void CmdRobotWorker::onStateChanged(QAbstractSocket::SocketState socketState ){
    qDebug()<< "(Robot" << robotName << ") The state of the socket changed :" << socketState;
}

void CmdRobotWorker::pingSlot(){
    missedPing = MISSED_PING_TIMER;
}


void CmdRobotWorker::changeRobotNameSlot(QString name){
    qDebug()<< "(Robot" << robotName << ") Changed the name of the robot to" << name;
    robotName = name;
}
