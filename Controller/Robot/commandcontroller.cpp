#include "commandcontroller.h"
#include <QDebug>
#include <QStringList>

CommandController::CommandController(QObject* parent, QString _ip, QString _robotName)
    : QObject(parent), ip(_ip), robotName(_robotName), cmdQueue(QList<QString>()), waitingForAnswer(false)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(cmdFinished()));
    timer.setSingleShot(true);
}

void CommandController::sendCommand(const QString cmd){
    if(!waitingForAnswer){
        waitingForAnswer = true;
        emit sendCommandSignal(cmd);
        timer.start(15000);
    } else {
        qDebug() << "CommandController::sendCommand got a cmd but already processing => sent to the queue" << cmdQueue;
        cmdQueue.append(cmd);
    }
    /// Send to the robot model to display the busy indicator when we are waiting for a cmd answer
    emit processingCmd(ip, waitingForAnswer);
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
                    emit setMessageTop(2, "Updated the name of the robot \"" + robotName + "\" to \"" + list.at(2) + "\"");
                    robotName = list.at(2);
                break;
                /*case 'b':
                    /// Changed the wifi information of a robot
                    /// NOT USED ANYMORE
                    Q_UNREACHABLE();
                break;*/
                case 'c':
                    /// Sent the robot to a new goal
                    /// nothing is needed on the qml side
                break;
                case 'd':
                    /// Paused the path of the robot
//                    emit setMessageTop(2, "Mission on pause");
                    emit updatePlayingPath(ip, false);
                break;
                case 'e':
                    /// Played the scan of the map
                    emit playedScanning(ip);
                break;
                case 'f':
                    /// Paused the scan of the map
                    emit pausedScanning(ip);
                break;
                case 'g':
                    /// starts an automatic scan
                break;
                /*
                case 'h':
                    /// Sent the ports to the robot
                    /// OSEF
                break;*/
                case 'i':
                    /// Sent a new path to the robot
                    list.removeFirst();
                    list.removeFirst();
                    emit setMessageTop(2, "Updated the path of \"" + robotName + "\"");
                    emit updatePath(ip, QStringList(list));
                break;
                case 'j':
                    /// Played the path of the robot
                    emit setMessageTop(2, "Robot \"" + robotName + "\" is starting its mission");
                    emit updatePlayingPath(ip, true);
                break;
                /*case 'k':
                    /// Deleted the path of the robot
                    /// NOT USED ANYMORE
                    Q_UNREACHABLE();
                break;*/
                case 'l':
                    /// Stopped the path of the robot
                    emit setMessageTop(2, "End of the robot \"" + robotName + "\" mission");
                    emit updatePlayingPath(ip, false);
                break;
                case 'm':
                    /// Stopped and deleted the path of the robot
                    emit stoppedDeletedPath(ip);
                    emit setMessageTop(2, "Deleted the path of \"" + robotName + "\"");
                break;
                case 'n':
                    /// Sent the new home to the robot
                    emit updateHome(ip, list.at(2).toDouble(), list.at(3).toDouble(), list.at(4).toDouble());
                    emit setMessageTop(2, "The robot " + robotName + " has a new home");
                break;
                case 'o':
                    /// Started the docking process
                    qDebug() << "CommandController::cmdAnswerSlot Started docking" << list;
                break;
                case 'p':
                    /// Stopped the docking process
                    emit setMessageTop(2, "Robot \"" + robotName + "\" is going to its charging station");
                    qDebug() << "CommandController::cmdAnswerSlot Stopped docking" << list;
                break;
                case 'q':
                    /// Started the laser of the robot
                    emit updateLaser(ip, true);
                break;
                case 'r':
                    /// Stopped the laser of the robot
                    emit updateLaser(ip, false);
                break;
                case 's':
                    /// Received the map from the robot
                    /// OSEF
                break;
                case 't':
                    /// Started a new scan
                    emit startedScanning(ip);
                break;
                case 'u':
                    /// Stopped the current scan
                    emit stoppedScanning(ip);
                break;
                /*case 'v':
                    /// Started to recover the position of a robot
                break;
                case 'w':
                    /// Stopped to recover the position of a robot
                break;*/
                case ',':
                    /// Play exploration
                    emit playedExploration(ip);
                break;
                case '.':
                    /// Pause exploration
                    emit pausedExploration(ip);
                break;
                case '/':
                    /// Pause exploration
                    emit setLooping(ip, list.at(2).toInt());
                break;
                default:
                    /// Unknown/unused command
                    qDebug() << "CommandController::cmdAnswerSlot Unknown command" << list;
                    Q_UNREACHABLE();
                break;
            }
        } else {
            qDebug() << "CommandController::cmdAnswerSlot The command failed or the robot is busy : " << list;
            /// TODO debug or handle the busy case
            //Q_UNREACHABLE();
        }
    } else {
        qDebug() << "CommandController::cmdAnswerSlot Did not get enough data : " << list;
        Q_UNREACHABLE();
    }

    qDebug() << "CommandController::cmdAnswerSlot in" << 15000 - timer.remainingTime();

    timer.stop();
    cmdFinished();
}

void CommandController::cmdFinished(){
    qDebug() << "CommandController::cmdFinished";
    waitingForAnswer = false;
    /// if the command queue is not empty we process the next command
    if(!cmdQueue.isEmpty()){
        qDebug() << "CommandController::cmdAnswerSlot got an answer and processing the next cmd in the queue" << cmdQueue;
        waitingForAnswer = true;
        emit sendCommandSignal(cmdQueue.takeFirst());
    }
    /// Send to the robot model to display the busy indicator when we are waiting for a cmd answer
    emit processingCmd(ip, waitingForAnswer);
}
