#include "robotpositionworker.h"
#include <QThread>

RobotPositionWorker::RobotPositionWorker(const QString newipAddress, const int newPort){
    ipAddress = newipAddress;
    port = newPort;
}

RobotPositionWorker::~RobotPositionWorker(){
    stopWorker();
}

void RobotPositionWorker::stopWorker(){
    if(socket && socket->isOpen())
        socket->close();
}

void RobotPositionWorker::connectSocket(){

    qDebug() << "(Robot pos thread" << ipAddress << ") Running";

    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()),this, SLOT(disconnectedSlot()));
    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));
    /// Connect the signal when an error occurs with the socket, to react accordingly
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    /// We try to connect to the robot, if an error occur,
    /// the errorConnectionSlot will try to reconnect
    socket->connectToHost(ipAddress, port);

    qDebug() << "(Robot pos thread" << ipAddress << ") connectSocket done";
}

void RobotPositionWorker::readTcpDataSlot(){
    QString data = socket->readAll();
    QRegExp rx("[ ]");

    /// Data is received as a string containing values separated by a space ("width height resolution originX originY")
    QStringList list = data.split(rx, QString::SkipEmptyParts);

    emit valueChangedRobot(ipAddress, list.at(0).toDouble(), list.at(1).toDouble(), list.at(2).toDouble());
}

void RobotPositionWorker::disconnectedSlot(){
    qDebug() << "(Robot pos thread" << ipAddress << ") Disconnected";
}

void RobotPositionWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    //qDebug() << "(RobotPositionWorker) Error while connecting :" << error;
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "(RobotPositionWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        break;
    case(QAbstractSocket::HostNotFoundError):
        qDebug() << "(RobotPositionWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        qDebug() << "(RobotPositionWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        qDebug() << "(RobotPositionWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        qDebug() << "(RobotPositionWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        qDebug() << "(RobotPositionWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        qDebug() << "(RobotPositionWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        qDebug() << "(RobotPositionWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        qDebug() << "(RobotPositionWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        qDebug() << "(RobotPositionWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        qDebug() << "(RobotPositionWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        qDebug() << "(RobotPositionWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        qDebug() << "(RobotPositionWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        qDebug() << "(RobotPositionWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        qDebug() << "(RobotPositionWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        qDebug() << "(RobotPositionWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        qDebug() << "(RobotPositionWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        qDebug() << "(RobotPositionWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        qDebug() << "(RobotPositionWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        qDebug() << "(RobotPositionWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        qDebug() << "(RobotPositionWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        qDebug() << "(RobotPositionWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        qDebug() << "(RobotPositionWorker) An unidentified error occurred.";
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "(RobotPositionWorker) Not supposed to be here.";
        break;
    }
}
