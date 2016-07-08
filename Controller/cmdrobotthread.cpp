#include "cmdrobotthread.h"
#include <QTime>
#include <QCoreApplication>


CmdRobotThread::CmdRobotThread(const QString newipAddress, const int newPort, const QString _robotName, QObject* parent) : QThread(parent){
    ipAddress = newipAddress;
    port = newPort;
    robotName = _robotName;
    connected = false;
    commandAnswer = "";
    missedPing = MISSED_PING_TIMER;
}

void CmdRobotThread::run(){
    qDebug() << "(Robot" << robotName << ") Command Thread launched";

    socketCmd = std::shared_ptr<QTcpSocket>(new QTcpSocket());

    //int qtype1 = qRegisterMetaType<QAbstractSocket::SocketError>("SocketError");

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socketCmd), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()) );
    /// Connect the signal hostFound which trigger when we find the host
    //connect(&(*socketCmd), SIGNAL(hostFound()), SLOT(hostFoundSlot()) );
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

    /// Throw an error if bytes are available but we can't read them
    while (socketCmd->bytesAvailable() < (int)sizeof(quint16) && !isInterruptionRequested()) {
        if (!socketCmd->waitForReadyRead()) {
            qDebug() << "(Robot" << robotName << ") Ready read error : " << socketCmd->errorString();
            socketCmd->close();
            exit();
            return;
        }
    }
    qDebug() << "(Robot" << robotName << ") Done";
    while(!isInterruptionRequested()){
        if(missedPing <= 0){
            emit robotIsDead(robotName, ipAddress);
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

    socketCmd->waitForBytesWritten();

    if(nbDataSend == -1){
        qDebug() << "(Robot" << robotName << ") An error occured while sending data";
    } else {
        qDebug() << "(Robot" << robotName << ") " << nbDataSend << " bytes sent";

        if(QString(cmd.at(0)).compare("e")){
            socketCmd->waitForReadyRead();
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
    while(!commandAnswer.compare("") && waitTime < 3){
        delay(500);
        waitTime++;
    }
    qDebug() << "Got answer and waited for" << (waitTime*500) << "seconds :" << commandAnswer;
    return commandAnswer;
}

void CmdRobotThread::hostFoundSlot(){
    qDebug() << "(Robot" << robotName << ") Host found";
}

void CmdRobotThread::connectedSlot(){
    qDebug() << "(Robot" << robotName << ") Connected";
    connected = true;
}

void CmdRobotThread::disconnectedSlot(){
    qDebug() << "(Robot" << robotName << ") Disconnected";
    connected = false;
    emit robotIsDead(robotName, ipAddress);
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
    //qDebug()<< "(Robot" << robotName << ") missedPingSlot called";
    missedPing = MISSED_PING_TIMER;
}
