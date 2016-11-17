#include "commandcontroller.h"
#include "Model/robot.h"
#include "Model/map.h"
#include "Controller/mainwindow.h"
#include <QStringList>
#include <QFile>

CommandController::CommandController(QWidget *parent) : QObject(parent), messageBox(QPointer<CommandMessageBox>(new CommandMessageBox(parent))), robotName(""){
    messageBox->setWindowTitle("Processing a command");
    connect(messageBox, SIGNAL(hideBox()), this, SLOT(userStopped()));
}

bool CommandController::sendCommand(QPointer<Robot> robot, QString cmd){

    if(robotName.isEmpty() && !cmd.isEmpty()){
        cmdAnswer = "";
        robot->sendCommand(cmd);


        /// React accordingly to the answer : return true or false
        QRegExp rx("[ ]");

        /// Data are received as a string separated by a space ("cmd done" or "cmd failed")
        QStringList listCmd = cmd.split(rx, QString::SkipEmptyParts);


        if(listCmd.at(0).compare("b") == 0)
            return true;
        else {
            robotName = robot->getName();
            cmdName = listCmd.at(0);
        }

        return robotWaitForAnswer(listCmd);

    } else {
        qDebug() << "CommandController::sendCommand Robot" << robotName << "is already processing the command" << cmdName;
    }
    return false;

    //return true;
}

void CommandController::openMessageBox(QStringList listCmd){
    QString msg("");
    QString cmd = listCmd.at(0);
    switch (cmd.at(0).unicode()) {
    case 'a':
        msg = "Sending the new name : " + listCmd.at(1) + " to the robot";
        break;
    case 'b':
        msg = "Sending the new wifi : " + listCmd.at(1) + " to the robot";
        break;
    case 'c':
        msg = "Sending the robot to a new destination : " + listCmd.at(1) + ", " + listCmd.at(2);
        break;
    case 'd':
        msg = "Pausing the path of the robot";
        break;
    case 'e':
        msg = "Starting the scan of the map";
        break;
    case 'f':
        msg = "Stopping the scan of the map";
        break;
    case 'g':
        msg = "Sending the map to the robot";
        break;
    case 'h':
        msg = "Sending the ports to the robot";
        break;
    case 'i':
        msg = "Sending a new path to the robot";
        break;
    case 'j':
        msg = "Playing the path of the robot";
        break;
    case 'k':
        msg = "Deleting the path of the robot";
        break;
    case 'l':
        msg = "Stopping the path of the robot";
        break;
    case 'm':
        msg = "Stopping and deleting the path of the robot";
        break;
    case 'n':
        msg = "Sending the new home to the robot";
        break;
    case 'o':
        msg = "Sending the robot to its home";
        break;
    case 'p':
        msg = "Stopping the robot to go home";
        break;
    default:
        msg = "Unknown command";
        break;
    }

    messageBox->setText(msg);
    messageBox->show();
}

bool CommandController::robotWaitForAnswer(QStringList listCmd){

    QRegExp rx("[ ]");
    openMessageBox(listCmd);

    qDebug() << "CommandController::robotWaitForAnswer Going to wait for an answer";
    while(cmdAnswer.compare("") == 0){
        qDebug() << "CommandController::robotWaitForAnswer waiting for an answer";
        MainWindow::delay(1000);
    }

    qDebug() << "The answer is :" << cmdAnswer;

    /// Data are received as a string separated by a space ("cmd done" or "cmd failed")
    QStringList list = cmdAnswer.split(rx, QString::SkipEmptyParts);
    cmdAnswer = "";


    if(list.size() == 2){
        if(list.at(0).compare(cmdName) == 0){
            robotName = "";
            cmdName = "";
            if(list.at(1).compare("done") == 0){
                qDebug() << "CommandController::robotWaitForAnswer The command" << cmdName << "succeeded";
                return true;
            } else if(list.at(1).compare("failed") == 0){
                qDebug() << "CommandController::robotWaitForAnswer The command" << cmdName << "failed";
                return false;
            } else {
                qDebug() << "CommandController::robotWaitForAnswer Got an answer to the right command but with an unknwon result :" << list;
                return false;
            }
        } else {
            /// Should be catch by cmdAnswerSlot
            qDebug() << "CommandController::robotWaitForAnswer Got an answer to the wrong command :" << list;
            return robotWaitForAnswer(listCmd);
        }
    } else {
        /// Should be catch by cmdAnswerSlot
        qDebug() << "CommandController::robotWaitForAnswer Got a wrong answer :" << list;
        return robotWaitForAnswer(listCmd);
    }
}

void CommandController::cmdAnswerSlot(QString answer){
    qDebug() << "CommandController::cmdAnswerSlot received answer :" << answer;
    cmdAnswer = "";

    /// React accordingly to the answer : return true or false
    QRegExp rx("[ ]");

    /// Data are received as a string separated by a space ("cmd done" or "cmd failed")
    QStringList list = answer.split(rx, QString::SkipEmptyParts);

    if(list.size() == 2){
        if(list.at(0).compare(cmdName) == 0){
            cmdAnswer = answer;
            messageBox->hide();
        } else {
            qDebug() << "CommandController::robotWaitForAnswer Got an answer to the wrong command :" << list;
        }
    } else {
        qDebug() << "CommandController::robotWaitForAnswer Got a wrong answer :" << list;
    }
}


void CommandController::sendNewMapToRobot(QPointer<Robot> robot, QString mapId, QSharedPointer<Map> map){
    qDebug() << "sendNewMapToRobot called on" << robot->getName() << "at ip" << robot->getIp() << "sending map id :" << mapId;

/*
    if(robotName.isEmpty()){
        cmdAnswer = "";

        qDebug() << "Sending the new map id to the robot" << robot->getName();
        /// Push the map id to send
        QByteArray byteArray;
        byteArray.push_back(mapId.toUtf8());
        byteArray.push_back(';');

        /// Push the map metadata to send
        QString mapMetadata = QString::number(map->getWidth()) + ' ' + QString::number(map->getHeight()) +
                ' ' + QString::number(map->getResolution()) + ' ' + QString::number(map->getOrigin().getX()) +
                ' ' + QString::number(map->getOrigin().getY());

        byteArray.push_back(mapMetadata.toUtf8());
        byteArray.push_back(';');

        /// Push the map to send
        QFile file(QString(GOBOT_PATH) + QString(MAP_FILE));
        if (!file.open(QIODevice::ReadOnly)) return;
        QByteArray blob = file.readAll();
        byteArray.push_back(blob);


        qDebug() << "Setting the map ID to the robot" << robot->getName();
        robot->setMapId(QUuid(mapId));

        robot->sendNewMap(byteArray);

        openMessageBox("Tmp Message");

        qDebug() << "CommandController::sendNewMapToRobot Going to wait while sending the map";
        while(cmdAnswer.compare("") == 0){
            qDebug() << "CommandController::sendNewMapToRobot waiting to finish to send the map";
            MainWindow::delay(1000);
        }
        qDebug() << "The answer is :" << cmdAnswer;

        cmdAnswer = "";
    } else {
        qDebug() << "CommandController::sendNewMapToRobot Already processing a command";
    }*/
}

void CommandController::robotDisconnected(QString _robotName){
    if(_robotName.compare(robotName)){
        qDebug() << "The robot" << robotName << " was waiting for an answer to the command" << cmdName << "but disconnected";
        cmdAnswer = "cmd failed";
    }
}

void CommandController::userStopped(){
    qDebug() << "The user pressed a button to stop to wait";
    cmdAnswer = "cmd failed";
}
