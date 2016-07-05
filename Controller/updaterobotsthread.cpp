#include "updaterobotsthread.h"
#include <QTcpServer>
#include <QTime>
#include <QCoreApplication>


UpdateRobotsThread::UpdateRobotsThread(const int newPort){
    qDebug() << "(UpdateRobotsThread) Thread launched";
    port = newPort;
    server = new QTcpServer(this);
    connect(server, SIGNAL(newConnection()), this, SLOT(newConnectionSlot()));
    connect(server, SIGNAL(acceptError(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    socket = new QTcpSocket(server);
    connect(socket, SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));
}

UpdateRobotsThread::~UpdateRobotsThread(){
    delete socket;
    delete server;
}

void UpdateRobotsThread::run(){
    qDebug() << "(UpdateRobotsThread) Waiting on port" << port;
    while(!this->isInterruptionRequested()){
        if(!server->isListening())
            server->listen(QHostAddress::Any, port);
        delay(300);
    }
}

void UpdateRobotsThread::newConnectionSlot(){
    socket = server->nextPendingConnection();
    connect(socket, SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));

    if(socket->state() == QTcpSocket::ConnectedState){
        qDebug() << "\n(UpdateRobotsThread) New connection established :" << socket->peerAddress().toString();
    } else {
        qDebug() << "\n(UpdateRobotsThread) Error : New connection could not be established :" << socket->peerAddress().toString();
    }

    int nbDataSend = socket->write("OK");

    socket->waitForBytesWritten();

    if(nbDataSend == -1){
        qDebug() << "(UpdateRobotsThread) An error occured while sending data";
    } else {
        //qDebug() << "(UpdateRobotsThread) " << nbDataSend << " bytes sent";
        socket->waitForReadyRead();
    }
    //qDebug() << "(UpdateRobotsThread) Closing the socket";
    socket->close();
    //qDebug() << "(UpdateRobotsThread) Socket closed";
}

void UpdateRobotsThread::disconnectedSlot(){
    qDebug() << "(UpdateRobotsThread) Disconnected from :" << socket->peerAddress().toString();
}

void UpdateRobotsThread::readTcpDataSlot(){
    QString hostname = socket->readAll();
    qDebug() << "(UpdateRobotsThread) Hostname received :" << hostname;
}

void UpdateRobotsThread::delay(const int ms) const{
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void UpdateRobotsThread::errorConnectionSlot(QAbstractSocket::SocketError error){
    qDebug() << "(UpdateRobotsThread) Error while connecting :" << error;
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        qDebug() << "The connection was refused by the peer (or timed out).";
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        break;
    case(QAbstractSocket::HostNotFoundError):
        qDebug() << "The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        qDebug() << "The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        qDebug() << "The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        qDebug() << "The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        qDebug() << "The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        qDebug() << "An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        qDebug() << "The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        qDebug() << "The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        qDebug() << "The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        qDebug() << "The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        qDebug() << "The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        qDebug() << "Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        qDebug() << "Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        qDebug() << "The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        qDebug() << "The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        qDebug() << "The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        qDebug() << "The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        qDebug() << "An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        qDebug() << "The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        qDebug() << "Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        qDebug() << "A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        qDebug() << "An unidentified error occurred.";
        break;
    default:
        qDebug() << "Not supposed to be here.";
        break;
    }
}
