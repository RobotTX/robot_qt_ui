#include "scanrobotthread.h"


ScanRobotThread::ScanRobotThread(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
}

void ScanRobotThread::run(){
    qDebug() << "Robot Thread";

    socketRobot = std::shared_ptr<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socketRobot), SIGNAL(readyRead()), SLOT(readTcpData()) );
    /// Connect the signal hostFound which trigger when we find the host
    //connect( socketRobot, SIGNAL(hostFound()), SLOT(hostFoundSlot()) );
    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socketRobot), SIGNAL(connected()), SLOT(connectedSlot()) );
    //connect( socketRobot, SIGNAL(error(QAbstractSocket::SocketError)), SLOT(errorSlot(QAbstractSocket::SocketError)) );
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socketRobot), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));

    /// Connect to the host
    socketRobot->connectToHost(ipAddress, port);

    int i = 1;
    while(!socketRobot->waitForConnected(5000)){
        //qDebug() << "(Robot) Attempt " << i << " :\nConnecting error : " << socketRobot->errorString();
        socketRobot->connectToHost(ipAddress, port);
        sleep(1);
        if(i++ > 100){
            qDebug() << "(Robot) Too much connection attempts";
            socketRobot -> close();
            exit();
            return;
        }
    }

    /// Throw an error if bytes are available but we can't read them
    while (socketRobot->bytesAvailable() < (int)sizeof(quint16)) {
        if (!socketRobot->waitForReadyRead()) {
            qDebug() << "(Robot) Ready read error : " << socketRobot->errorString();
            socketRobot -> close();
            exit();
            return;
        }
    }
}

void ScanRobotThread::readTcpData(){
    QString data = socketRobot->readAll();
    QRegExp rx("[ ]");
    QStringList list = data.split(rx, QString::SkipEmptyParts);

    /// Data are received as a string separated by a space ("width height resolution originX originY")
    emit valueChangedRobot(list.at(0).toDouble(), list.at(1).toDouble(), list.at(2).toDouble());

}

void ScanRobotThread::hostFoundSlot(){
    qDebug() << "(Robot) Host found";
}

void ScanRobotThread::connectedSlot(){
    qDebug() << "(Robot) Connected";
}

void ScanRobotThread::disconnectedSlot(){
    qDebug() << "(Map) Disconnected";
}

void ScanRobotThread::errorSlot(QAbstractSocket::SocketError error){
    qDebug() << "(Robot) Error : " << error;
}
