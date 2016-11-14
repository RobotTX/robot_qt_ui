#include "cmdrobotworker.h"


CmdRobotWorker::CmdRobotWorker(const QString _ipAddress, const int cmdPort, const int _metadataPort, const int _robotPort, const int _mapPort, const QString _robotName){
    ipAddress = _ipAddress;
    port = cmdPort;
    robotName = _robotName;
    metadataPort = _metadataPort;
    robotPort = _robotPort;
    mapPort = _mapPort;
    timeCounter = 0;
}

CmdRobotWorker::~CmdRobotWorker(){
    stopCmdRobotWorkerSlot();
}

void CmdRobotWorker::stopCmdRobotWorkerSlot(){
    if(socket && socket->isOpen())
        socket->close();
}

void CmdRobotWorker::connectSocket(){

    //qDebug() << "(Robot" << robotName << ") Command Thread launched";

    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));
    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socket), SIGNAL(connected()), this, SLOT(connectedSlot()));
    //connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));


    while(socket->state() != QAbstractSocket::ConnectedState){
        if(socket->state() == QAbstractSocket::UnconnectedState){
            socket->connectToHost(ipAddress, port);
            socket->waitForConnected(10000);
            /*if(!socket->waitForConnected(10000)){
                qDebug() << "(Robot" << robotName << ") Connecting error : " << socket->errorString() << " Trying again.";
            }*/
        }
    }
    qDebug() << "(Robot" << robotName << ") We should be connected";
}

void CmdRobotWorker::sendCommand(const QString cmd){
    qDebug() << "(Robot" << robotName << ") Command to send :" << cmd << "to" << ipAddress << "at port " << port;
    int nbDataSend = socket->write(QString(cmd + " } ").toUtf8());

    socket->waitForBytesWritten(100);

    if(nbDataSend == -1){
        qDebug() << "(Robot" << robotName << ") An error occured while sending data";
    } else {
        qDebug() << "(Robot" << robotName << ") " << nbDataSend << " bytes sent";
    }
    qDebug() << "(Robot" << robotName << ") Command sent successfully";
}

void CmdRobotWorker::readTcpDataSlot(){
    QString commandAnswer = socket->readAll();
    qDebug() << "(Robot" << robotName << ") readTcpDataSlot :" << commandAnswer;
    emit cmdAnswer(commandAnswer);
}

void CmdRobotWorker::connectedSlot(){
    qDebug() << "(Robot" << robotName << ") Connected";

    QString portStr = "h \"" + QString::number(metadataPort) + "\" \"" + QString::number(robotPort) + "\" \"" + QString::number(mapPort) + "\" } ";
    qDebug() << "(Robot" << robotName << ") Sending ports : " << portStr;
    bool tmpBool(false);
    while(!tmpBool){
        socket->write(portStr.toUtf8());
        if(socket->waitForBytesWritten(100)){
            qDebug() << "(Robot" << robotName << ") Ports sent";
            tmpBool = true;
            emit portSent();
        } else {
            qDebug() << "(Robot" << robotName << ") Ports could not be sent";
        };
    }

    timer.setInterval(1000);
    connect(&timer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    timer.start();
}

void CmdRobotWorker::disconnectedSlot(){
    qDebug() << "(Robot" << robotName << ") Disconnected at ip" << ipAddress;
    if(robotName.compare("") != 0){
        qDebug() << "(Robot" << robotName << ") Emitting robotIsDead";
        timer.stop();
        timeCounter = 0;
        emit robotIsDead(robotName, ipAddress);
        if(socket->isOpen())
            socket->close();
    }
}

void CmdRobotWorker::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Robot" << robotName << ") Catched a connection error : " << error;
}

void CmdRobotWorker::onStateChanged(QAbstractSocket::SocketState socketState ){
    qDebug() << "(Robot" << robotName << ") The state of the socket changed :" << socketState;
}

void CmdRobotWorker::pingSlot(void){
    qDebug()<< "(Robot" << robotName << ") Received the ping";
    timer.start();
    timeCounter = 0;
}

void CmdRobotWorker::timerSlot(void){
    timeCounter++;
    qDebug()<< "(Robot" << robotName << ") Did not receive any ping from this robot for" << timeCounter << "seconds";
    if(timeCounter >= ROBOT_TIMER && socket->isOpen())
        socket->close();
}

void CmdRobotWorker::changeRobotNameSlot(QString name){
    qDebug()<< "(Robot" << robotName << ") Changed the name of the robot to" << name;
    robotName = name;
}
