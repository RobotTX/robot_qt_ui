#include "robotserverworker.h"
#include <QTcpServer>
#include <QCoreApplication>
#include <QRegExp>
#include <string>


RobotServerWorker::RobotServerWorker(const int newPort, QObject* parent): QTcpServer(parent), port(newPort) {
    connect(this, SIGNAL(newConnection()), this, SLOT(newConnectionSlot()));
    connect(this, SIGNAL(acceptError(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));
    startServer();
}

void RobotServerWorker::startServer(){
    if(listen(QHostAddress::Any, port) == 0)
        // qDebug() << "(RobotServerWorker) Server failed to listen on port" << port;
    {} else {}
        // qDebug() << "Server is listening on port" << port;
}

void RobotServerWorker::stopWorker(){
    close();
}

void RobotServerWorker::newConnectionSlot(){
    /// server awaits for a new connection
    QTcpSocket* socket = this->nextPendingConnection();

    /// if the server gets a new connection
    if(socket->state() == QTcpSocket::ConnectedState){

        socket->write("OK\n");
        socket->waitForReadyRead();
        QString str = socket->readAll();

        QStringList strList = str.split(QChar(31), QString::SkipEmptyParts);

        if(strList.size() == 6){
            //// qDebug() << "(RobotServerWorker) robotIsAlive" << strList;
            /// name, ip, pathstage, battery, charging, docking status
            emit robotIsAlive(strList.at(0), socket->peerAddress().toString().remove(0, 7),
                              static_cast<QString> (strList.at(1)).toInt(),
                              static_cast<QString> (strList.at(2)).toInt(),
                              static_cast<QString> (strList.at(3)).toInt(),
                              static_cast<QString> (strList.at(4)).toInt(),
                              static_cast<QString> (strList.at(5)).toInt(),
                              false);
        }
        else if(strList.size() == 7){
            //// qDebug() << "(RobotServerWorker) robotIsAlive" << strList;
            /// name, ip, pathstage, battery, charging, docking status
            emit robotIsAlive(strList.at(0), socket->peerAddress().toString().remove(0, 7),
                              static_cast<QString> (strList.at(1)).toInt(),
                              static_cast<QString> (strList.at(2)).toInt(),
                              static_cast<QString> (strList.at(3)).toInt(),
                              static_cast<QString> (strList.at(4)).toInt(),
                              static_cast<QString> (strList.at(5)).toInt(),
                              static_cast<QString> (strList.at(6)).toInt());
        }
        else {}
            // qDebug() << "(RobotServerWorker) Not enough param received for robotIsAlive" << strList;
    }

    socket->close();
}

void RobotServerWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    // qDebug() << "(RobotServerWorker) Error while connecting :" << error;
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        /// if the connection has been refused we symply try again after a short sleep
        // qDebug() << "(RobotServerWorker) The connection was refused by the peer (or timed out).";
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        // qDebug() << "(RobotServerWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        break;
    case(QAbstractSocket::HostNotFoundError):
        // qDebug() << "(RobotServerWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        // qDebug() << "(RobotServerWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        // qDebug() << "(RobotServerWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        // qDebug() << "(RobotServerWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        // qDebug() << "(RobotServerWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        // qDebug() << "(RobotServerWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        // qDebug() << "(RobotServerWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        // qDebug() << "(RobotServerWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        // qDebug() << "(RobotServerWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        // qDebug() << "(RobotServerWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        // qDebug() << "(RobotServerWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        // qDebug() << "(RobotServerWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        // qDebug() << "(RobotServerWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        // qDebug() << "(RobotServerWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        // qDebug() << "(RobotServerWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        // qDebug() << "(RobotServerWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        // qDebug() << "(RobotServerWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        // qDebug() << "(RobotServerWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        // qDebug() << "(RobotServerWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        // qDebug() << "(RobotServerWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        // qDebug() << "(RobotServerWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        // qDebug() << "(RobotServerWorker) An unidentified error occurred.";
        break;
    default:
        /// NOTE can probably remove that when testing phase is over
        Q_UNREACHABLE();
        // qDebug() << "(RobotServerWorker) Not supposed to be here.";
        break;
    }
}
