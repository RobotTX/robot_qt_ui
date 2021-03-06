#include "sendnewmapworker.h"
#include <QThread>
#include <assert.h>
#include <fstream>
#include <string>
#include <iostream>

using namespace std;

SendNewMapWorker::SendNewMapWorker(const QString _ipAddress, const int _port): ipAddress(_ipAddress), port(_port) {}

SendNewMapWorker::~SendNewMapWorker(){
    stopWorker();
}

void SendNewMapWorker::stopWorker(){
    if(socket && socket->isOpen()) {
        socket->close();
        qDebug() << "socket is close";
    }
}

void SendNewMapWorker::connectSocket(){
     qDebug() << "(Robot new map thread" << ipAddress << ") Running";

    socket = QPointer<QTcpSocket>(new QTcpSocket());

    /// Connect the signal readyRead which tell us when data arrived to the function that treat them
    connect(&(*socket), SIGNAL(readyRead()), this, SLOT(readTcpDataSlot()));
    /// Connect the signal when an error occurs with the socket, to react accordingly
    connect(&(*socket), SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorConnectionSlot(QAbstractSocket::SocketError)));

    /// We try to connect to the robot, if an error occur,
    /// the errorConnectionSlot will try to reconnect
    socket->connectToHost(ipAddress, port);

     qDebug() << "(New Map) connectSocket done" << ipAddress << port;
}


void SendNewMapWorker::readTcpDataSlot(){
    QString dataStr = socket->readAll();
    // qDebug() << "(New Map) data received :" << ipAddress << dataStr;
    QStringList strList = dataStr.split(" ", QString::SkipEmptyParts);
    bool deleteHomePath = false;
    if(strList.size() == 2)
        deleteHomePath = QString(strList.at(1)).toInt();
    emit doneSendingNewMapSignal(deleteHomePath);
}

void SendNewMapWorker::writeTcpDataMP3Slot(QString fileName) {
    QVector<char> send_msg = readSoundFile(fileName);
//    qDebug() << "audio size = " << send_msg.size() << endl;
    QByteArray byteArray;
//    qDebug() << "send size " << send_msg.size() << endl;

    QByteArray toSend = QByteArray().append(static_cast<char*>(send_msg.data()), send_msg.size());

    toSend.push_back('!');
    toSend.push_back('@');
    toSend.push_back('#');
    toSend.push_back('$');

    byteArray.append(toSend);

//    qDebug() << "byteArray.size() = " << byteArray.size();

//    qDebug() << "Adress = " << ipAddress;

    if(socket && socket->isOpen()){
        int nbDataSend = socket->write(byteArray);

        socket->waitForBytesWritten();

        qDebug() << "ipAdress = " << ipAddress;
        qDebug() << "nbDataSend = " << nbDataSend;

        if(nbDataSend == -1)
             qDebug() << "(MP3) An error occured while sending data" << ipAddress;
        else
             qDebug() << "(MP3) " << ipAddress << ":" << nbDataSend << "bytes sent out of" << byteArray.size();
    } else {
        /// NOTE: what to do if the socket is closed ? can it happen ?
         qDebug() << "(MP3) Trying to write on a socket that is not created or connected yet" << ipAddress;
        Q_UNREACHABLE();
    }

}

QVector<char> SendNewMapWorker::readSoundFile(QString path){
    std::ifstream sourcestr(path.toStdString(), std::ios::in | std::ios::binary);
    long size;
    char * buffer;

    if(!sourcestr){
        qDebug() <<"can not open "<< path << endl;
        return {};
    }

    // get file size using buffer's members
    size=sourcestr.rdbuf()->pubseekoff (0,ios::end,ios::in);
    sourcestr.rdbuf()->pubseekpos (0,ios::in);

    // allocate memory to contain file data
    buffer=new char[size];

    // get file data
    sourcestr.rdbuf()->sgetn (buffer,size);

    //close mp3 file
    sourcestr.close();

    //save the data in vector
    QVector<char> result;
    for(int i=0;i<size;i++)
        result.push_back(buffer[i]);

    //return mp3 data
    return result;
}

void SendNewMapWorker::writeTcpDataSlot(QString mapId, QString date, QString metadata, QImage map){
    // qDebug() << "(New Map) Sending the new map to" << ipAddress << "at port " << port;

    QByteArray byteArray;

    /// Push the map id to send
    byteArray.push_back(mapId.toUtf8());
    byteArray.push_back(';');

    /// Push the date to send
    byteArray.push_back(date.toUtf8());
    byteArray.push_back(';');

    /// Push the map metadata to send
    byteArray.push_back(metadata.toUtf8());
    byteArray.push_back(';');

    map.save("/home/m-a/Desktop/pute.pgm", "PGM");

    QByteArray mapArray;
    /// Compress and push the map to send
    int last = 205;
    uint32_t count = 0;
    for(int i = 0; i < map.height(); i++){
        for(int j = 0; j < map.width(); j++){
            int curr = map.pixelColor(j, i).red();
            if(last != curr && count != 0){
                /// count being an int, we divide it in 4 bytes to push in the QByteArray
                mapArray.push_back(last);
                mapArray.push_back(static_cast<uint8_t>((count & 0xff000000) >> 24));
                mapArray.push_back(static_cast<uint8_t>((count & 0x00ff0000) >> 16));
                mapArray.push_back(static_cast<uint8_t>((count & 0x0000ff00) >> 8));
                mapArray.push_back(static_cast<uint8_t>(count & 0x000000ff));
                last = curr;
                count = 0;
            }
            count++;
        }
    }

    mapArray.push_back(last);
    mapArray.push_back(static_cast<uint8_t>((count & 0xff000000) >> 24));
    mapArray.push_back(static_cast<uint8_t>((count & 0x00ff0000) >> 16));
    mapArray.push_back(static_cast<uint8_t>((count & 0x0000ff00) >> 8));
    mapArray.push_back(static_cast<uint8_t>(count & 0x000000ff));

    mapArray.push_back(static_cast<uint8_t>(254));
    mapArray.push_back(static_cast<uint8_t>(254));
    mapArray.push_back(static_cast<uint8_t>(254));
    mapArray.push_back(static_cast<uint8_t>(254));
    mapArray.push_back(static_cast<uint8_t>(254));

    byteArray.append(mapArray);

    // qDebug() << "(New Map) Map size to send :" << ipAddress << mapArray.length();

    if(socket && socket->isOpen()){
        int nbDataSend = socket->write(byteArray);

        socket->waitForBytesWritten();
        qDebug() << "ipAdress = " << ipAddress;

        if(nbDataSend == -1)
            // qDebug() << "(New Map) An error occured while sending data" << ipAddress;
        {} else {}
            // qDebug() << "(New Map) " << ipAddress << ":" << nbDataSend << "bytes sent out of" << byteArray.size();
    } else {
        /// NOTE: what to do if the socket is closed ? can it happen ?
         qDebug() << "(New Map) Trying to write on a socket that is not created or connected yet" << ipAddress;
        Q_UNREACHABLE();
    }
}

void SendNewMapWorker::errorConnectionSlot(QAbstractSocket::SocketError error){
    switch (error) {
    case(QAbstractSocket::ConnectionRefusedError):
        /// if the connection has been refused we symply try again after a short sleep
        ///
        QThread::sleep(1);
        socket->connectToHost(ipAddress, port);
        break;
    case(QAbstractSocket::RemoteHostClosedError):
        // qDebug() << "(SendNewMapWorker) The remote host closed the connection. Note that the client socket (i.e., this socket) will be closed after the remote close notification has been sent.";
        emit robotIsDead();
        break;
    case(QAbstractSocket::HostNotFoundError):
        // qDebug() << "(SendNewMapWorker) The host address was not found.";
        break;
    case(QAbstractSocket::SocketAccessError):
        // qDebug() << "(SendNewMapWorker) The socket operation failed because the application lacked the required privileges.";
        break;
    case(QAbstractSocket::SocketResourceError):
        // qDebug() << "(SendNewMapWorker) The local system ran out of resources (e.g., too many sockets).";
        break;
    case(QAbstractSocket::SocketTimeoutError):
        // qDebug() << "(SendNewMapWorker) The socket operation timed out.";
        break;
    case(QAbstractSocket::DatagramTooLargeError):
        // qDebug() << "(SendNewMapWorker) The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
        break;
    case(QAbstractSocket::NetworkError):
        // qDebug() << "(SendNewMapWorker) An error occurred with the network (e.g., the network cable was accidentally plugged out).";
        break;
    case(QAbstractSocket::AddressInUseError):
        // qDebug() << "(SendNewMapWorker) The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
        break;
    case(QAbstractSocket::SocketAddressNotAvailableError):
        // qDebug() << "(SendNewMapWorker) The address specified to QAbstractSocket::bind() does not belong to the host.";
        break;
    case(QAbstractSocket::UnsupportedSocketOperationError):
        // qDebug() << "(SendNewMapWorker) The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
        break;
    case(QAbstractSocket::ProxyAuthenticationRequiredError):
        // qDebug() << "(SendNewMapWorker) The socket is using a proxy, and the proxy requires authentication.";
        break;
    case(QAbstractSocket::SslHandshakeFailedError):
        // qDebug() << "(SendNewMapWorker) The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
        break;
    case(QAbstractSocket::UnfinishedSocketOperationError):
        // qDebug() << "(SendNewMapWorker) Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
        break;
    case(QAbstractSocket::ProxyConnectionRefusedError):
        // qDebug() << "(SendNewMapWorker) Could not contact the proxy server because the connection to that server was denied";
        break;
    case(QAbstractSocket::ProxyConnectionClosedError):
        // qDebug() << "(SendNewMapWorker) The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
        break;
    case(QAbstractSocket::ProxyConnectionTimeoutError):
        // qDebug() << "(SendNewMapWorker) The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
        break;
    case(QAbstractSocket::ProxyNotFoundError):
        // qDebug() << "(SendNewMapWorker) The proxy address set with setProxy() (or the application proxy) was not found.";
        break;
    case(QAbstractSocket::ProxyProtocolError):
        // qDebug() << "(SendNewMapWorker) The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
        break;
    case(QAbstractSocket::OperationError):
        // qDebug() << "(SendNewMapWorker) An operation was attempted while the socket was in a state that did not permit it.";
        break;
    case(QAbstractSocket::SslInternalError):
        // qDebug() << "(SendNewMapWorker) The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
        break;
    case(QAbstractSocket::SslInvalidUserDataError):
        // qDebug() << "(SendNewMapWorker) Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
        break;
    case(QAbstractSocket::TemporaryError):
        // qDebug() << "(SendNewMapWorker) A temporary error occurred (e.g., operation would block and socket is non-blocking).";
        break;
    case(QAbstractSocket::UnknownSocketError):
        // qDebug() << "(SendNewMapWorker) An unidentified error occurred.";
        break;
    default:
        /// NOTE can probably remove that when testing phase is over
        Q_UNREACHABLE();
        // qDebug() << "(SendNewMapWorker) Not supposed to be here.";
        break;
    }
}
