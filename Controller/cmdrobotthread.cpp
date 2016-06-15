#include "cmdrobotthread.h"

CmdRobotThread::CmdRobotThread(const QString newipAddress, const int newPort, const QString _robotName){
    ipAddress = newipAddress;
    port = newPort;
    robotName = _robotName;
    connected = false;
}

void CmdRobotThread::run(){
    qDebug() << "Command Thread";

    socketCmd = std::shared_ptr<QTcpSocket>(new QTcpSocket());

    /// Connect the signal hostFound which trigger when we find the host
    //connect( socketCmd, SIGNAL(hostFound()), SLOT(hostFoundSlot()) );
    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socketCmd), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socketCmd, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socketCmd), SIGNAL(disconnected()), SLOT(disconnectedSlot()));

    /// Connect to the host
    socketCmd->connectToHost(ipAddress, port);

    int i = 1;
    while(!socketCmd->waitForConnected(5000)){
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
}

bool CmdRobotThread::sendCommand(QString cmd){
    qDebug() << "(Robot" << robotName << ") Command to send : " << cmd << "to " << ipAddress << "at port " << port;

    if(connected){
        int nbDataSend = socketCmd->write(QString(cmd + " } ").toUtf8());

        socketCmd->waitForBytesWritten();

        if(nbDataSend == -1){
            qDebug() << "(Robot" << robotName << ") An error occured while sending data";
        } else {
            qDebug() << "(Robot" << robotName << ") " << nbDataSend << " bytes sent";
        }
        return true;
    } else {
        qDebug() << "(Robot" << robotName << ") Error : Robot at ip" << ipAddress << ": " << port << "not connected";
        return false;
    }
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
}

void CmdRobotThread::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Robot" << robotName << ") Error : " << error;
}
