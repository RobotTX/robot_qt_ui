#include "commandcontroller.h"
#include <QStringList>
#include <QFile>
#include <QFocusEvent>
#include <QApplication>
#include "Helper/helper.h"
#include "Controller/mainwindow.h"
#include "Model/Robots/robot.h"
#include "Model/Map/map.h"

CommandController::CommandController(QWidget *parent)
    : QObject(parent), robotName(""), messageBox(parent), stop(false), newRobotName(""),
      groupName(""), pathName(""), scan(false), nb(-1), path(QStringList()) {
    messageBox.setWindowTitle("Processing a command");
    connect(&messageBox, SIGNAL(hideBox()), this, SLOT(commandFailed()));
}

bool CommandController::sendCommand(QPointer<Robot> robot, QString cmd,
                                    QString _newRobotName, QString _groupName,
                                    QString _pathName, bool _scan,
                                    int _nb, QStringList _path){

    qDebug() << "CommandController::sendCommand" << cmd << "to" << robot->getName();
    /// if the command is not already being processed
    if(robotName.isEmpty() && !cmd.isEmpty() && !stop){


        /// React accordingly to the answer : return true or false
        QRegExp rx("[ ]");

        /// Data is received as a string separated by a space ("cmd done" or "cmd failed")
        QStringList listCmd = cmd.split(rx, QString::SkipEmptyParts);


        if(TESTING){
            robotName = robot->getName();
            cmdName = listCmd.at(0);
            groupName = _groupName;
            pathName = _pathName;
            scan = _scan;
            nb = _nb;
            newRobotName = _newRobotName;
            path = _path;
            qDebug() << "CommandController::sendCommand" << robotName << cmdName << newRobotName << groupName << pathName << scan << nb << path;

            cmdAnswerSlot(listCmd.at(0)+" done");
            return true;
        } else {
            if(robot->isScanning() && listCmd.at(0).compare("c") != 0 && listCmd.at(0).compare("e") != 0 && listCmd.at(0).compare("f") != 0 &&
                        listCmd.at(0).compare("t") != 0 && listCmd.at(0).compare("u") != 0){
                qDebug() << "CommandController::sendCommand Robot" << robot->getName() << "is already scanning and can not perform command" << cmd;
                return false;
            }

            robot->sendCommand(cmd);

            robotName = robot->getName();
            cmdName = listCmd.at(0);
            groupName = _groupName;
            pathName = _pathName;
            scan = _scan;
            nb = _nb;
            newRobotName = _newRobotName;
            path = _path;
            qDebug() << "CommandController::sendCommand" << robotName << cmdName << newRobotName << groupName << pathName << scan << nb << path;

            openMessageBox(listCmd);
            return true;
        }
    } else {
        qDebug() << "CommandController::sendCommand Robot" << robotName << "is already processing the command" << cmdName;
        return false;
    }
}

void CommandController::cmdAnswerSlot(QString answer){
    qDebug() << "CommandController::cmdAnswerSlot received answer :" << answer;

    /// React accordingly to the answer : return true or false
    QRegExp rx("[ ]");

    /// Data is received as a string separated by a space ("cmd done" or "cmd failed")
    QStringList list = answer.split(rx, QString::SkipEmptyParts);

    bool success = false;

    if(list.size() == 2){
        if(list.at(0).compare(cmdName) == 0){
            messageBox.hide();
            if(list.at(1).compare("done") == 0){
                qDebug() << "CommandController::cmdAnswerSlot The command" << cmdName << "succeeded";
                success = true;
            } else if(list.at(1).compare("failed") == 0){
                qDebug() << "CommandController::cmdAnswerSlot The command" << cmdName << "failed";
            } else {
                qDebug() << "CommandController::cmdAnswerSlot Got an answer to the right command but with an unknwon result :" << list;
            }
        } else if(list.at(0).compare("cmd") == 0 && list.at(1).compare("failed") == 0){
            qDebug() << "CommandController::cmdAnswerSlot The user stopped the command";
        } else
            /// Should be caught by cmdAnswerSlot
            qDebug() << "CommandController::cmdAnswerSlot Got an answer to the wrong command :" << list;
    } else {
        /// Should be caught by cmdAnswerSlot
        qDebug() << "CommandController::cmdAnswerSlot Got a wrong answer :" << list;
        Q_UNREACHABLE();
    }

    emit commandDone(cmdName, success, robotName, newRobotName, groupName, pathName, scan, nb, path);
    resetParams();
}

void CommandController::resetParams(){
    robotName = "";
    cmdName = "";
    groupName = "";
    pathName = "";
    scan = false;
    nb = -1;
    newRobotName = "";
    path = QStringList();
}

void CommandController::robotDisconnected(QString _robotName){
    qDebug() << "CommandController::robotDisconnected The robot" << _robotName << "disconnected but the robot" << robotName << "was waiting";

    if(_robotName.compare(robotName))
        commandFailed();
}

void CommandController::commandFailed(){
    messageBox.hide();
    emit commandDone(cmdName, false, robotName, newRobotName, groupName, pathName, scan, nb, path);
    resetParams();
}

void CommandController::stopAllCommand(){
    stop = true;
    commandFailed();
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
        msg = "Playing the scan of the map";
        break;
    case 'f':
        msg = "Pausing the scan of the map";
        break;
    case 'g':
        msg = "Sending the new name : " + listCmd.at(1) + " to the robot and the new wifi : " + listCmd.at(2) + " to the robot";
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
        /// NOT USED
        msg = "Stopping the robot to go home";
        Q_UNREACHABLE();
        break;
    case 'q':
        msg = "Starting the laser of the robot";
        break;
    case 'r':
        msg = "Stoping the laser of the robot";
        break;
    case 's':
        msg = "Receiving the map from the robot";
        break;
    case 't':
        msg = "Starting a new scan";
        break;
    case 'u':
        msg = "Stopping the current scan";
        break;
    default:
        msg = "Unknown command " + cmd.at(0).unicode();
        break;
    }

    messageBox.setText(msg);
    /**
    /// is supposed to reset the timer
    messageBox.hide();
    */
    //messageBox.setWindowFlags(Qt::WindowStaysOnTopHint);

    //messageBox.show();
    //messageBox.raise();
    //messageBox.activateWindow();
    /// TODO check that it's not stopping the other commands
    messageBox.exec();
}
