#include "sendnewmapworker.h"
#include <QThread>

SendNewMapWorker::SendNewMapWorker(const QString _ipAddress, const int _port){
    ipAddress = _ipAddress;
    port = _port;
}

SendNewMapWorker::~SendNewMapWorker(){
    stopThread();
}

void SendNewMapWorker::stopThread(){
    if(socket && socket->isOpen())
        socket->close();
}

void SendNewMapWorker::connectSocket(){
    qDebug() << "(Robot new map thread" << ipAddress << ") Running";

    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal connected which trigger when we are connected to the host
    //connect(&(*socket), SIGNAL(connected()), this, SLOT(connectedSlot()));
    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    socket->connectToHost(ipAddress, port);
    qDebug() << "(New Map) connectSocket done";
}

void SendNewMapWorker::readTcpDataSlot(){
    QString dataStr = socket->readAll();
    qDebug() << "(New Map) data received :" << dataStr;
    emit doneSendingNewMapSignal();
}

void SendNewMapWorker::writeTcpDataSlot(QByteArray cmd){
    qDebug() << "(New Map) Sending the new map to" << ipAddress << "at port " << port;

    cmd.push_back((int8_t) -2);

    int nbDataSend = socket->write(cmd);

    socket->waitForBytesWritten();

    if(nbDataSend == -1){
        qDebug() << "(New Map) An error occured while sending data";
    } else {
        qDebug() << "(New Map) " << nbDataSend << "bytes sent out of" << cmd.size();
    }
}

void SendNewMapWorker::disconnectedSlot(){
    qDebug() << "(New Map) Disconnected at ip" << ipAddress;
}

void SendNewMapWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    //qDebug() << "(SendNewMapWorker) Error while connecting :" << error;
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        //qDebug() << "(SendNewMapWorker) The connection was refused by the peer (or timed out).";
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "(SendNewMapWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        break;
    case(QAbstractSocket::HostNotFoundError):
        qDebug() << "(SendNewMapWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        qDebug() << "(SendNewMapWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        qDebug() << "(SendNewMapWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        qDebug() << "(SendNewMapWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        qDebug() << "(SendNewMapWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        qDebug() << "(SendNewMapWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        qDebug() << "(SendNewMapWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        qDebug() << "(SendNewMapWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        qDebug() << "(SendNewMapWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        qDebug() << "(SendNewMapWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        qDebug() << "(SendNewMapWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        qDebug() << "(SendNewMapWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        qDebug() << "(SendNewMapWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        qDebug() << "(SendNewMapWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        qDebug() << "(SendNewMapWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        qDebug() << "(SendNewMapWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        qDebug() << "(SendNewMapWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        qDebug() << "(SendNewMapWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        qDebug() << "(SendNewMapWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        qDebug() << "(SendNewMapWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        qDebug() << "(SendNewMapWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        qDebug() << "(SendNewMapWorker) An unidentified error occurred.";
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "(SendNewMapWorker) Not supposed to be here.";
        break;
    }
}
