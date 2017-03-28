#include "commandcontroller.h"
#include <QDebug>

CommandController::CommandController(QObject* parent) : QObject(parent) {

}

void CommandController::sendCommand(const QString cmd){
    qDebug() << "(RobotController) Send command called" << cmd;
    emit sendCommandSignal(cmd);
}


void CommandController::cmdAnswerSlot(QString answer){
    qDebug() << "RobotController::cmdAnswerSlot called" << answer;
}
