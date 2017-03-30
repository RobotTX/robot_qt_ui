#include "commandcontroller.h"
#include <QDebug>
#include <QStringList>

CommandController::CommandController(QObject* parent, QString _ip) : QObject(parent), ip(_ip) {

}

void CommandController::sendCommand(const QString cmd){
    /// TODO gerer multi command etc
    emit sendCommandSignal(cmd);
}


void CommandController::cmdAnswerSlot(QString answer){
    QList<QString> list = answer.split(QChar(31), QString::SkipEmptyParts);
    qDebug() << "CommandController::cmdAnswerSlot called" << list;

    if(list.size() > 1){
        if(list.at(0).compare("done") == 0){
            switch (list.at(1).at(0).unicode()) {
                case 'a':
                    /// Changed the name of the robot
                    emit updateName(ip, list.at(2));
                break;
                /*case 'b':
                    /// Changed the wifi informations of a robot
                    /// NOT USED ANYMORE
                    Q_UNREACHABLE();
                break;
                case 'c':
                    /// Sent the robot to a new goal
                    /// OSEF
                break;*/
                case 'd':
                    /// Paused the path of the robot
                    emit updatePlayingPath(ip, false);
                break;
                /*case 'e':
                    /// Played the scan of the map
                break;
                case 'f':
                    /// Paused the scan of the map
                break;
                case 'g':
                    /// Updated the name & wifi of the robot
                    /// NOT USED ANYMORE
                    Q_UNREACHABLE();
                break;
                case 'h':
                    /// Sent the ports to the robot
                    /// OSEF
                break;*/
                case 'i':
                    /// Sent a new path to the robot
                    list.removeFirst();
                    list.removeFirst();
                    emit updatePath(ip, QStringList(list));
                break;
                case 'j':
                    /// Played the path of the robot
                    emit updatePlayingPath(ip, true);
                break;
                /*case 'k':
                    /// Deleted the path of the robot
                    /// NOT USED ANYMORE
                    Q_UNREACHABLE();
                break;*/
                case 'l':
                    /// Stopped the path of the robot
                    emit updatePlayingPath(ip, false);
                break;
                case 'm':
                    /// Stopped and deleted the path of the robot
                    emit stoppedDeletedPath(ip);
                break;
                case 'n':
                    /// Sent the new home to the robot
                    emit updateHome(ip, list.at(2), list.at(3).toFloat(), list.at(4).toFloat());
                break;
                /*case 'o':
                    /// Sent the robot to its home
                break;
                case 'p':
                    /// Stopped the robot to go home
                    /// NOT USED ANYMORE
                    Q_UNREACHABLE();
                break;
                case 'q':
                    /// Started the laser of the robot
                    /// OSEF
                break;
                case 'r':
                    /// Stopped the laser of the robot
                    /// OSEF
                break;*/
                case 's':
                    /// Received the map from the robot
                    /// OSEF
                break;
                /*case 't':
                    /// Started a new scan
                break;
                case 'u':
                    /// Stopped the current scan
                break;
                case 'v':
                    /// Started to recover the position of a robot
                break;
                case 'w':
                    /// Stopped to recover the position of a robot
                break;*/
                default:
                    /// Unknown/unused command
                    qDebug() << "CommandController::cmdAnswerSlot Unknown command" << list;
                    Q_UNREACHABLE();
                break;
            }
        } else {
            qDebug() << "CommandController::cmdAnswerSlot The command failed : " << list;
            Q_UNREACHABLE();
        }
    } else {
        qDebug() << "CommandController::cmdAnswerSlot Did not get enough data : " << list;
        Q_UNREACHABLE();
    }
}
