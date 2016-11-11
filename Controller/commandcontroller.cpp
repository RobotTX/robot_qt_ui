#include "commandcontroller.h"
#include "Model/robot.h"
#include "Model/map.h"
#include "Controller/mainwindow.h"
#include <QStringList>
#include <QFile>

CommandController::CommandController(QWidget *parent) : QObject(parent), messageBox(new CommandMessageBox(parent)), robotName(""){
    messageBox->setWindowTitle("Processing a command");
    connect(messageBox, SIGNAL(hideBox()), this, SLOT(userStopped()));
}

bool CommandController::sendCommand(Robot* robot, QString cmd){
    /*if(robotName.isEmpty() && !cmd.isEmpty()){
        cmdAnswer = "";
        robot->sendCommand(cmd);


        /// React accordingly to the answer : return true or false
        QRegExp rx("[ ]");

        /// Data are received as a string separated by a space ("cmd done" or "cmd failed")
        QStringList listCmd = cmd.split(rx, QString::SkipEmptyParts);


        if(listCmd.at(0).compare("b") == 0)
            return true;
        else
            robotName = robot->getName();

        /// TODO customize message
        robotWaitForAnswer(listCmd.at(0));

        qDebug() << "CommandController::sendCommand Going to wait for an answer";
        while(cmdAnswer.compare("") == 0){
            qDebug() << "CommandController::sendCommand waiting for an answer";
            MainWindow::delay(1000);
        }

        qDebug() << "The answer is :" << cmdAnswer;

        /// Data are received as a string separated by a space ("cmd done" or "cmd failed")
        QStringList list = cmdAnswer.split(rx, QString::SkipEmptyParts);
        cmdAnswer = "";
        robotName = "";

        /// TODO message if failed + depends on which cmd failed, or received another msg
        if(list.size() == 2){
            if(list.at(1).compare("done") == 0)
                return true;
        }
        qDebug() << "CommandController::sendCommand Got a wrong answer or a fail (need to custom msg)";
    } else {
        qDebug() << "CommandController::sendCommand Robot" << robotName << "is already processing a command";
    }
    return false;*/

    return true;
}

void CommandController::robotWaitForAnswer(QString msg){
    messageBox->setText(msg);
    messageBox->show();
}

void CommandController::cmdAnswerSlot(QString answer){
    qDebug() << "CommandController::cmdAnswerSlot received answer :" << answer;
    cmdAnswer = "";

    /// React accordingly to the answer : return true or false
    QRegExp rx("[ ]");

    /// Data are received as a string separated by a space ("cmd done" or "cmd failed")
    QStringList list = answer.split(rx, QString::SkipEmptyParts);

    if(list.size() == 2){
        /// If the message box is opened, we were expecting an answer
        if(messageBox && !robotName.isEmpty()){
            cmdAnswer = answer;
            messageBox->hide();
        } else {
            qDebug() << "CommandController::cmdAnswerSlot received a message that is not from a command";
        }
    } else {
        qDebug() << "CommandController::cmdAnswerSlot received a message that is not from a command";
    }
}


void CommandController::sendNewMapToRobot(Robot* robot, QString mapId, QSharedPointer<Map> map){
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

        robotWaitForAnswer("Tmp Message");

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
        qDebug() << "The robot we sent a command to, just disconnected";
        cmdAnswer = "cmd failed";
    }
}

void CommandController::userStopped(){
    qDebug() << "The user pressed a button to stop to wait";
    /// TODO send to the robot to stop ?
    cmdAnswer = "cmd failed";
}
