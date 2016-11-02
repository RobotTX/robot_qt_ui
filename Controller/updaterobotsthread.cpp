#include "updaterobotsthread.h"
#include <QTcpServer>
#include <QCoreApplication>
#include <QRegExp>


UpdateRobotsThread::UpdateRobotsThread(const int newPort){
    //qDebug() << "(UpdateRobotsThread) Thread launched";
    port = newPort;
    server = new QTcpServer(this);
    connect(server, SIGNAL(newConnection()), this, SLOT(newConnectionSlot()));
    connect(server, SIGNAL(acceptError(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));
}

UpdateRobotsThread::~UpdateRobotsThread(){
    delete server;
}

void UpdateRobotsThread::run(){
    if(server->listen(QHostAddress::Any, port) == 0)
        qDebug() << "(UpdateRobotsThread) Server listen failed";
    else
        exec();
}

void UpdateRobotsThread::newConnectionSlot(){
    QTcpSocket* socket = server->nextPendingConnection();

    if(socket->state() == QTcpSocket::ConnectedState){
        //qDebug() << "\n\n(UpdateRobotsThread) New connection established :" << socket->peerAddress().toString();

        socket->write("OK");
        socket->waitForReadyRead();
        QString str = socket->readAll();

        QStringList strList = str.split("\"", QString::SkipEmptyParts);

        if(strList.size() > 1){
            //qDebug() << "(UpdateRobotsThread)" << strList;
            emit robotIsAlive(strList.at(0), socket->peerAddress().toString(), strList.at(1), strList.at(2), std::stoi(strList.at(3).toStdString()));
        } else {
            qDebug() << "(UpdateRobotsThread) Not enough param received for robotIsAlive";
        }

    }

    socket->close();
}

void UpdateRobotsThread::errorConnectionSlot(QAbstractSocket::SocketError error){
    qDebug() << "(UpdateRobotsThread) Error while connecting :" << error;
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        qDebug() << "(UpdateRobotsThread) The connection was refused by the peer (or timed out).";
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "(UpdateRobotsThread) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        break;
    case(QAbstractSocket::HostNotFoundError):
        qDebug() << "(UpdateRobotsThread) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        qDebug() << "(UpdateRobotsThread) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        qDebug() << "(UpdateRobotsThread) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        qDebug() << "(UpdateRobotsThread) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        qDebug() << "(UpdateRobotsThread) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        qDebug() << "(UpdateRobotsThread) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        qDebug() << "(UpdateRobotsThread) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        qDebug() << "(UpdateRobotsThread) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        qDebug() << "(UpdateRobotsThread) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        qDebug() << "(UpdateRobotsThread) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        qDebug() << "(UpdateRobotsThread) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        qDebug() << "(UpdateRobotsThread) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        qDebug() << "(UpdateRobotsThread) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        qDebug() << "(UpdateRobotsThread) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        qDebug() << "(UpdateRobotsThread) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        qDebug() << "(UpdateRobotsThread) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        qDebug() << "(UpdateRobotsThread) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        qDebug() << "(UpdateRobotsThread) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        qDebug() << "(UpdateRobotsThread) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        qDebug() << "(UpdateRobotsThread) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        qDebug() << "(UpdateRobotsThread) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        qDebug() << "(UpdateRobotsThread) An unidentified error occurred.";
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "(UpdateRobotsThread) Not supposed to be here.";
        break;
    }
}
