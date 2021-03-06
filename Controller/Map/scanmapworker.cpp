#include "scanmapworker.h"
#include <QDataStream>
#include <QFile>
#include <QStringList>

ScanMapWorker::ScanMapWorker(const QString newipAddress, const int newPort):
    ipAddress(newipAddress), port(newPort), data(QByteArray())
{}

ScanMapWorker::~ScanMapWorker(){
    stopWorker();
}

void ScanMapWorker::stopWorker(){
    if(socket && socket->isOpen())
        socket->close();
}

void ScanMapWorker::connectSocket(){
    // qDebug() << "(ScanMapWorker) Trying to connect to" << ipAddress << "at port" << port;

    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));

    /// Connect the signal when an error occurs with the socket, to react accordingly
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    /// We try to connect to the robot, if an error occurs,
    /// the errorConnectionSlot will try to reconnect
    socket->connectToHost(ipAddress, port);

     qDebug() << "(ScanMapWorker) connectSocket done port " << port;
}

void ScanMapWorker::readTcpDataSlot(){
    data.append(socket->readAll());
    // qDebug() << "(ScanMapWorker) Received data" << data.size();

    QString mapId("");
    QString mapDate("");
    QString resolution("-1");
    QString originX("-1");
    QString originY("-1");
    uint32_t map_width(0);
    uint32_t map_height(0);

    /// The TCP protocol sending blocks of data, a map is defined by a random number
    /// of blocks, so we wait till the last byte of a block is -2, meaning we have received
    /// a complete map
    if(data.size() >= 5 && static_cast<uint8_t>(data.at(data.size()-5)) == 254 && static_cast<uint8_t>(data.at(data.size()-4)) == 254
            && static_cast<uint8_t>(data.at(data.size()-3)) == 254 && static_cast<uint8_t>(data.at(data.size()-2)) == 254 &&
            (static_cast<uint8_t>(data.at(data.size()-1)) == 251 || static_cast<uint8_t>(data.at(data.size()-1)) == 252
             || static_cast<uint8_t>(data.at(data.size()-1)) == 253 || static_cast<uint8_t>(data.at(data.size()-1)) == 254)){

        /// Emit the signal valueChangedMap, meaning that we finished to receive a whole map
        /// and we can display it
        /// who:
        /// 0 : scan
        /// 1 : application requesting at connection time
        /// 2 : to merge
        int who = 0;
        if(static_cast<uint8_t>(data.at(data.size()-1)) == 254)
            who = 1;
//        else if(static_cast<uint8_t>(data.at(data.size()-1)) == 252)
//            who = 2;

//        // qDebug() << "(ScanMapWorker) Who :" << who;

        QString mapInfo("");
        if(who == 0){
            int i = 0;
            bool gotMapInfo = false;
            while(!gotMapInfo && i < data.size() - 6){
                if(static_cast<uint8_t>(data.at(i)) == 252 && static_cast<uint8_t>(data.at(i+1)) == 252
                        && static_cast<uint8_t>(data.at(i+2)) == 252 && static_cast<uint8_t>(data.at(i+3)) == 252 &&
                        static_cast<uint8_t>(data.at(i+4)) == 252)
                    gotMapInfo = true;
                else
                    mapInfo.append(static_cast<char>(data.at(i)));
                i++;
            }

            data.remove(0, mapInfo.size() + 5);

//            // qDebug() << "(ScanMapWorker) Got mapInfo (who = 0) :" << mapInfo;
            QStringList strList = mapInfo.split(" ", QString::SkipEmptyParts);
            if(strList.size() > 4){
                map_width = QString(strList.at(0)).toInt();
                map_height = QString(strList.at(1)).toInt();
                resolution = strList.at(2);
                originX = strList.at(3);
                originY = strList.at(4);
            } else {}
                // qDebug() << "(ScanMapWorker) Could not parse mapInfo :" << mapInfo;

        } else if(who == 1 || who == 2){
            /// If the map comes from a pgm, we get the mapId and mapDate associated to it
            int i = 0;
            bool gotMapInfo = false;
            while(!gotMapInfo && i < data.size() - 6){
                if(static_cast<uint8_t>(data.at(i)) == 252 && static_cast<uint8_t>(data.at(i+1)) == 252
                        && static_cast<uint8_t>(data.at(i+2)) == 252 && static_cast<uint8_t>(data.at(i+3)) == 252 &&
                        static_cast<uint8_t>(data.at(i+4)) == 252)
                    gotMapInfo = true;
                else
                    mapInfo.append(static_cast<char>(data.at(i)));
                i++;
            }

            data.remove(0, mapInfo.size() + 5);

//            // qDebug() << "(ScanMapWorker) Got mapInfo (who = 1 or 2) :" << mapInfo;
            QStringList strList = mapInfo.split(" ", QString::SkipEmptyParts);
            if(strList.size() > 6){
                mapId = strList.at(0);
                mapDate = strList.at(1);
                map_width = QString(strList.at(2)).toInt();
                map_height = QString(strList.at(3)).toInt();
                resolution = strList.at(4);
                originX = strList.at(5);
                originY = strList.at(6);
            } else {}
                // qDebug() << "(ScanMapWorker) Could not parse mapInfo :" << mapInfo;

        }

        /// Remove the end bytes 254 254 254 254 254 as we no longer need them
        data.remove(data.size()-5, 5);

        emit valueChangedMap(data, who, mapId, mapDate, resolution, originX, originY, map_width, map_height);

        /// Clear the Vector that contain the map, once it has been processed
        data.clear();
    }
}

void ScanMapWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        /// if the connection has been refused we symply try again after a short sleep
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        // qDebug() << "(ScanMapWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        emit robotIsDead();
        break;
    case(QAbstractSocket::HostNotFoundError):
        // qDebug() << "(ScanMapWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        // qDebug() << "(ScanMapWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        // qDebug() << "(ScanMapWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        // qDebug() << "(ScanMapWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        // qDebug() << "(ScanMapWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        // qDebug() << "(ScanMapWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        // qDebug() << "(ScanMapWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        // qDebug() << "(ScanMapWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        // qDebug() << "(ScanMapWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        // qDebug() << "(ScanMapWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        // qDebug() << "(ScanMapWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        // qDebug() << "(ScanMapWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        // qDebug() << "(ScanMapWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        // qDebug() << "(ScanMapWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        // qDebug() << "(ScanMapWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        // qDebug() << "(ScanMapWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        // qDebug() << "(ScanMapWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        // qDebug() << "(ScanMapWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        // qDebug() << "(ScanMapWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        // qDebug() << "(ScanMapWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        // qDebug() << "(ScanMapWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        // qDebug() << "(ScanMapWorker) An unidentified error occurred.";
        break;
    default:
        /// NOTE can probably remove that when testing phase is over
        Q_UNREACHABLE();
        // qDebug() << "(ScanMapWorker) Not supposed to be here.";
        break;
    }
}
