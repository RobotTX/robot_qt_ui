#include "localmapworker.h"
#include <QDataStream>

LocalMapWorker::LocalMapWorker(const QString _ipAddress, const int _port): ipAddress(_ipAddress), port(_port), data(QByteArray()) {}

LocalMapWorker::~LocalMapWorker(){
    stopWorker();
}

void LocalMapWorker::connectSocket(){

    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal <readyRead> which tells us when data arrived to the function that process it
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));

    /// Connect the signal when an error occurs with the socket, to react accordingly
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    /// We try to connect to the robot, if an error occur,
    /// the errorConnectionSlot will try to reconnect
    socket->connectToHost(ipAddress, port);

    /// to add an entry for this robot in the obstacles map
    emit addNewRobotObstacles(ipAddress);

    qDebug() << "(Local Map Thread) connectSocket done";
}

void LocalMapWorker::stopWorker(){
    if(socket && socket->isOpen()){
        socket->close();
        delete socket;
    }
}

void LocalMapWorker::readTcpDataSlot(){
    data.append(socket->readAll());

    /// parameters of the laser scans
    float angle_min, angle_max, angle_increment;
    QVector<float> ranges;

    /// we construct the float values using 4 bytes at a time

    *(reinterpret_cast<uchar*>(&angle_min) + 3) = data.at(3);
    *(reinterpret_cast<uchar*>(&angle_min) + 2) = data.at(2);
    *(reinterpret_cast<uchar*>(&angle_min) + 1) = data.at(1);
    *(reinterpret_cast<uchar*>(&angle_min) + 0) = data.at(0);

    *(reinterpret_cast<uchar*>(&angle_max) + 3) = data.at(7);
    *(reinterpret_cast<uchar*>(&angle_max) + 2) = data.at(6);
    *(reinterpret_cast<uchar*>(&angle_max) + 1) = data.at(5);
    *(reinterpret_cast<uchar*>(&angle_max) + 0) = data.at(4);

    *(reinterpret_cast<uchar*>(&angle_increment) + 3) = data.at(11);
    *(reinterpret_cast<uchar*>(&angle_increment) + 2) = data.at(10);
    *(reinterpret_cast<uchar*>(&angle_increment) + 1) = data.at(9);
    *(reinterpret_cast<uchar*>(&angle_increment) + 0) = data.at(8);

    for(int i = 12; i < data.size(); i += 4){
        QByteArray currentValue = data.mid(i, 4);

        float range;

        *(reinterpret_cast<uchar*>(&range) + 3) = currentValue.at(3);
        *(reinterpret_cast<uchar*>(&range) + 2) = currentValue.at(2);
        *(reinterpret_cast<uchar*>(&range) + 1) = currentValue.at(1);
        *(reinterpret_cast<uchar*>(&range) + 0) = currentValue.at(0);

        /// the appended -1.0f at the end of the batch so when we reach it we ignore everything else we have
        if(range < 0)
            break;

        ranges.push_back(range);

    }

    data = QByteArray();

    qDebug() << "local map worker inside thread" << QThread::currentThreadId();

    /// sometimes we don't receive a complete scan and the first values do not correspond to the values of angle_min
    /// angle_max and angle_increment in which case we receive positive values instead which correspond to ranges
    /// that's why we check that the angle_min is negative before transmitting the data
    if(angle_min < 0)
        emit laserValues(angle_min, angle_max, angle_increment, ranges, ipAddress);
}

void LocalMapWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        /// if the connection has been refused we symply try again
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "(LocalMapWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        break;
    case(QAbstractSocket::HostNotFoundError):
        qDebug() << "(LocalMapWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        qDebug() << "(LocalMapWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        qDebug() << "(LocalMapWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        qDebug() << "(LocalMapWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        qDebug() << "(LocalMapWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        qDebug() << "(LocalMapWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        qDebug() << "(LocalMapWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        qDebug() << "(LocalMapWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        qDebug() << "(LocalMapWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        qDebug() << "(LocalMapWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        qDebug() << "(LocalMapWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        qDebug() << "(LocalMapWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        qDebug() << "(LocalMapWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        qDebug() << "(LocalMapWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        qDebug() << "(LocalMapWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        qDebug() << "(LocalMapWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        qDebug() << "(LocalMapWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        qDebug() << "(LocalMapWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        qDebug() << "(LocalMapWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        qDebug() << "(LocalMapWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        qDebug() << "(LocalMapWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        qDebug() << "(LocalMapWorker) An unidentified error occurred.";
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "(LocalMapWorker) Not supposed to be here.";
        break;
    }
}
