#include "cmdrobotworker.h"
#include <QThread>

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
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));


    socket->connectToHost(ipAddress, port);
    qDebug() << "(Robot" << robotName << ") connectSocket done";

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
    timer = new QTimer(this);

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

    timer->setInterval(1000);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    timer->start();
}

void CmdRobotWorker::disconnectedSlot(){
    qDebug() << "(Robot" << robotName << ") Disconnected at ip" << ipAddress;
    if(robotName.compare("") != 0){
        qDebug() << "(Robot" << robotName << ") Emitting robotIsDead";
        timer->stop();
        timeCounter = 0;
        emit robotIsDead(robotName, ipAddress);
        if(socket->isOpen())
            socket->close();
    }
}

void CmdRobotWorker::onStateChanged(QAbstractSocket::SocketState socketState ){
    qDebug() << "(Robot" << robotName << ") The state of the socket changed :" << socketState;
}

void CmdRobotWorker::pingSlot(void){
    qDebug()<< "(Robot" << robotName << ") Received the ping";
    timer->start();
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

void CmdRobotWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    //qDebug() << "(CmdRobotWorker) Error while connecting :" << error;
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        //qDebug() << "(CmdRobotWorker) The connection was refused by the peer (or timed out).";
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "(CmdRobotWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        break;
    case(QAbstractSocket::HostNotFoundError):
        qDebug() << "(CmdRobotWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        qDebug() << "(CmdRobotWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        qDebug() << "(CmdRobotWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        qDebug() << "(CmdRobotWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        qDebug() << "(CmdRobotWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        qDebug() << "(CmdRobotWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        qDebug() << "(CmdRobotWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        qDebug() << "(CmdRobotWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        qDebug() << "(CmdRobotWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        qDebug() << "(CmdRobotWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        qDebug() << "(CmdRobotWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        qDebug() << "(CmdRobotWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        qDebug() << "(CmdRobotWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        qDebug() << "(CmdRobotWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        qDebug() << "(CmdRobotWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        qDebug() << "(CmdRobotWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        qDebug() << "(CmdRobotWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        qDebug() << "(CmdRobotWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        qDebug() << "(CmdRobotWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        qDebug() << "(CmdRobotWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        qDebug() << "(CmdRobotWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        qDebug() << "(CmdRobotWorker) An unidentified error occurred.";
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "(CmdRobotWorker) Not supposed to be here.";
        break;
    }
}
