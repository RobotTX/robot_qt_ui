#include "cmdrobotthread.h"
#include <QTime>
#include <QCoreApplication>


CmdRobotThread::CmdRobotThread(const QString newipAddress, const int cmdPort, const int _metadataPort, const int _robotPort, const int _mapPort, const QString _robotName, QObject* parent) : QThread(parent){
    ipAddress = newipAddress;
    port = cmdPort;
    robotName = _robotName;
    connected = false;
    commandAnswer = "";
    missedPing = MISSED_PING_TIMER;
    metadataPort = _metadataPort;
    robotPort = _robotPort;
    mapPort = _mapPort;
}

void CmdRobotThread::run(){
    qDebug() << "(Robot" << robotName << ") Command Thread launched";

    socketCmd = std::shared_ptr<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socketCmd), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()) );
    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socketCmd), SIGNAL(connected()), this, SLOT(connectedSlot()) );
    //connect(&(*socketCmd), SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socketCmd), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));
    connect(this, SIGNAL(writeCommand(QString)), this, SLOT(writeCommandSlot(QString)));

    /// Connect to the host
    socketCmd->connectToHost(ipAddress, port);

    int i = 1;
    while(!socketCmd->waitForConnected(5000) && !isInterruptionRequested()){
        //qDebug() << "(Robot" << robotName << ") Attempt " << i << ":\nConnecting error : " << socketCmd->errorString();
        socketCmd->connectToHost(ipAddress, port);
        sleep(1);
        if(i++ > 100){
            qDebug() << "(Robot" << robotName << ") Too much connection attempts";
            socketCmd -> close();
            exit();
            return;
        }
    }

    //delay(2000);
    QString portStr = "h \"" + QString::number(metadataPort) + "\" \"" + QString::number(robotPort) + "\" \"" + QString::number(mapPort) + "\" } ";
    qDebug() << "(Robot" << robotName << ") Sending ports : " << portStr;
    socketCmd->write(portStr.toUtf8());

    socketCmd->waitForBytesWritten(1000);


    qDebug() << "(Robot" << robotName << ") Done";
    while(!isInterruptionRequested()){
        if(missedPing <= 0){
            emit robotIsDead(robotName, ipAddress);
            socketCmd -> close();
            exit();
            return;
        }
        //socketCmd->state();
        delay(500);
        missedPing--;
        qDebug() << "Received no ping during the last :" << (float) ((MISSED_PING_TIMER - missedPing)/2) << " seconds.";
    }
}

bool CmdRobotThread::sendCommand(const QString cmd){
    qDebug() << "(Robot" << robotName << ") Command to send : " << cmd << "to " << ipAddress << "at port " << port;

    if(connected){
        emit writeCommand(cmd);
        qDebug() << "(Robot" << robotName << ") Command sent successfully";
        return true;
    } else {
        qDebug() << "(Robot" << robotName << ") Error : Robot at ip" << ipAddress << ": " << port << "not connected";
        return false;
    }
}

void CmdRobotThread::writeCommandSlot(QString cmd){
    qDebug() << "writeCommandSlot called";
    int nbDataSend = socketCmd->write(QString(cmd + " } ").toUtf8());

    socketCmd->waitForBytesWritten(100);

    if(nbDataSend == -1){
        qDebug() << "(Robot" << robotName << ") An error occured while sending data";
    } else {
        qDebug() << "(Robot" << robotName << ") " << nbDataSend << " bytes sent";

        if(QString(cmd.at(0)).compare("e")){
            socketCmd->waitForReadyRead(100);
        } else {
            qDebug() << "No wait";
            delay(2000);
        }
    }
}

void CmdRobotThread::readTcpDataSlot(){
    commandAnswer = socketCmd->readAll();
    missedPing = MISSED_PING_TIMER;
    qDebug() << "(Robot" << robotName << ") readTcpDataSlot :" << commandAnswer;
}

QString CmdRobotThread::waitAnswer(){
    qDebug() << "waiting for an answer";
    int waitTime = 0;
    while(!commandAnswer.compare("") && waitTime < 20){
        delay(500);
        waitTime++;
    }

    if(waitTime >= 20){
        qDebug() << "Waited for an answer for too long";
    } else {
        qDebug() << "Got answer and waited for" << (waitTime/2) << "seconds :" << commandAnswer;
    }

    return commandAnswer;
}

void CmdRobotThread::connectedSlot(){
    qDebug() << "(Robot" << robotName << ") Connected";
    connected = true;
}

void CmdRobotThread::disconnectedSlot(){
    qDebug() << "(Robot" << robotName << ") Disconnected at ip" << ipAddress;
    connected = false;
    if(robotName.compare("") != 0){
        emit robotIsDead(robotName, ipAddress);
    }
}

void CmdRobotThread::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Robot" << robotName << ") Catched a connection error : " << error;
}

void CmdRobotThread::onStateChanged(QAbstractSocket::SocketState socketState ){
    qDebug()<< "(Robot" << robotName << ") The state of the socket changed :" << socketState;
}

void CmdRobotThread::delay(const int ms) const{
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void CmdRobotThread::pingSlot(){
    missedPing = MISSED_PING_TIMER;
}


void CmdRobotThread::changeRobotNameSlot(QString name){
    qDebug()<< "(Robot" << robotName << ") Changed the name of the robot to" << name;
    robotName = name;
}
