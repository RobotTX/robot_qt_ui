#include "particlecloudworker.h"
#include <QPointF>

ParticleCloudWorker::ParticleCloudWorker(const QString _ipAddress, const int _port): ipAddress(_ipAddress), port(_port) {
    qDebug() << "PARTICLECLOUDWORKER CREATED WITH PORT" << port;
}

ParticleCloudWorker::~ParticleCloudWorker(){
    stopWorker();
}

void ParticleCloudWorker::stopWorker(){
    if(socket && socket->isOpen()){
        socket->close();
        delete socket;
    }
}

void ParticleCloudWorker::connectSocket(){
    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal <readyRead> which tells us when data arrived to the function that process it
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));

    /// Connect the signal when an error occurs with the socket, to react accordingly
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    /// We try to connect to the robot, if an error occur,
    /// the errorConnectionSlot will try to reconnect
    socket->connectToHost(ipAddress, port);

    qDebug() << "(Particle Cloud Thread) connectSocket done";
}

void ParticleCloudWorker::readTcpDataSlot(){

    data.append(socket->readAll());

    QVector<QPointF> points;

    /// we construct the double values using 8 bytes at a time

    for(int i = 0; i < data.size(); i += 16){

        QByteArray currentValue = data.mid(i, 16);

        if(currentValue.size() > 15){

            double x;
            double y;

            /// x component of our point
            *(reinterpret_cast<uchar*>(&x) + 7) = currentValue.at(7);
            *(reinterpret_cast<uchar*>(&x) + 6) = currentValue.at(6);
            *(reinterpret_cast<uchar*>(&x) + 5) = currentValue.at(5);
            *(reinterpret_cast<uchar*>(&x) + 4) = currentValue.at(4);
            *(reinterpret_cast<uchar*>(&x) + 3) = currentValue.at(3);
            *(reinterpret_cast<uchar*>(&x) + 2) = currentValue.at(2);
            *(reinterpret_cast<uchar*>(&x) + 1) = currentValue.at(1);
            *(reinterpret_cast<uchar*>(&x) + 0) = currentValue.at(0);

            /// y component of our point
            *(reinterpret_cast<uchar*>(&y) + 7) = currentValue.at(15);
            *(reinterpret_cast<uchar*>(&y) + 6) = currentValue.at(14);
            *(reinterpret_cast<uchar*>(&y) + 5) = currentValue.at(13);
            *(reinterpret_cast<uchar*>(&y) + 4) = currentValue.at(12);
            *(reinterpret_cast<uchar*>(&y) + 3) = currentValue.at(11);
            *(reinterpret_cast<uchar*>(&y) + 2) = currentValue.at(10);
            *(reinterpret_cast<uchar*>(&y) + 1) = currentValue.at(9);
            *(reinterpret_cast<uchar*>(&y) + 0) = currentValue.at(8);

            points.push_back(QPointF(x, y));
        }

    }

    qDebug() << "Particle Cloud Worker points:";

    for(int i = 0; i < points.size(); i++)
        qDebug() << points.at(i).x() << points.at(i).y();

    data = QByteArray();

    /// TODO do something with those values
}


void ParticleCloudWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        /// if the connection has been refused we symply try again
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "(ParticleCloudWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        emit robotIsDead();
        break;
    case(QAbstractSocket::HostNotFoundError):
        qDebug() << "(ParticleCloudWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        qDebug() << "(ParticleCloudWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        qDebug() << "(ParticleCloudWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        qDebug() << "(ParticleCloudWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        qDebug() << "(ParticleCloudWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        qDebug() << "(ParticleCloudWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        qDebug() << "(ParticleCloudWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        qDebug() << "(ParticleCloudWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        qDebug() << "(ParticleCloudWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        qDebug() << "(ParticleCloudWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        qDebug() << "(ParticleCloudWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        qDebug() << "(ParticleCloudWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        qDebug() << "(ParticleCloudWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        qDebug() << "(ParticleCloudWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        qDebug() << "(ParticleCloudWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        qDebug() << "(ParticleCloudWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        qDebug() << "(ParticleCloudWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        qDebug() << "(ParticleCloudWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        qDebug() << "(ParticleCloudWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        qDebug() << "(ParticleCloudWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        qDebug() << "(ParticleCloudWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        qDebug() << "(ParticleCloudWorker) An unidentified error occurred.";
        break;
    default:
        /// NOTE can probably remove that when testing phase is over
        Q_UNREACHABLE();
        qDebug() << "(ParticleCloudWorker) Not supposed to be here.";
        break;
    }
}
