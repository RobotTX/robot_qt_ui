#include "backuprobotworker.h"
#include <QThread>

BackupRobotWorker::BackupRobotWorker(const QString _ipAddress, const int _port) : ipAddress(_ipAddress), port(_port) {}

void BackupRobotWorker::connectSocket(){

    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));

    /// Connect the signal connected which trigger when we are connected to the host
    connect(&(*socket), SIGNAL(connected()), this, SLOT(connectedSlot()));

    /// Connect the signal disconnected which trigger when we are disconnected from the host
    connect(&(*socket), SIGNAL(disconnected()), this, SLOT(disconnectedSlot()));

    /// Connect the signal when an error occurs with the socket, to react accordingly
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    /// We try to connect to the robot, if an error occurs,
    /// the errorConnectionSlot will try to reconnect
    socket->connectToHost(ipAddress, port);

    qDebug() << "backup system robot" << ipAddress << " connectSocket done";
}

BackupRobotWorker::~BackupRobotWorker(){
    stopWorker();
}

void BackupRobotWorker::connectedSlot(){
    qDebug() << "Backup system of Robot" << ipAddress << " Connected";
}

void BackupRobotWorker::disconnectedSlot(){
    qDebug() << "Backup system of Robot" << ipAddress << " Disconnected";
    /// Upon disconnection, we want to tell the MainWindow that we disconnected
    /// and close the socket
    emit backupSystemIsDown();
    if(socket->isOpen())
        socket->close();
}

void BackupRobotWorker::callForReboot(){
    qDebug() << "Calling for reboot of robot at address" << ipAddress;
    int nbDataSend = socket->write("reboot");

    socket->waitForBytesWritten(100);

    if(nbDataSend == -1)
        qDebug() << "Backup system at" << ipAddress << " could not be reached, failed to call for reboot";
    else
        qDebug() << "Backup system at" << ipAddress << "called for reboot";
}

void BackupRobotWorker::readTcpDataSlot(){
    qDebug() << "backup worker reading tcp data";
    QString ackMessage = socket->readAll();
    qDebug() << "Backup worker received msg :" << ackMessage;
    /// TODO message to user ? or simply whatever, only for troubleshooting
}

void BackupRobotWorker::stopWorker(){
    if(socket && socket->isOpen()){
        socket->close();
        delete socket;
    }
}

void BackupRobotWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        /// if the connection has been refused we symply try again after a short sleep
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        qDebug() << "(CmdRobotWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        emit backupSystemIsDown();
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
        /// NOTE can probably remove that when testing phase is over
        Q_UNREACHABLE();
        qDebug() << "(CmdRobotWorker) Not supposed to be here.";
        break;
    }
}
