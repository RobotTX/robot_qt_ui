#include "commandmessagebox.h"
#include <QTimer>
#include <QPushButton>
#include <QDebug>
#include <QCloseEvent>

CommandMessageBox::CommandMessageBox(QWidget *parent) : QMessageBox(parent){
    /// non-modal means it won't block other widgets from receiving slots while the msg box is opened
    setModal(false);
    abortButton = addButton(QMessageBox::Abort);
}

void CommandMessageBox::show(){
    abortButton->hide();

    /// Timer used to show the abort button when we wait for too long for an answer
    QTimer* timer = new QTimer(this);
    timer->setInterval(15000);
    timer->setSingleShot(true);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    timer->start();

    QMessageBox::show();
}

void CommandMessageBox::timerSlot(){
    abortButton->show();
}

void CommandMessageBox::done(int r){
    qDebug() << "CommandMessageBox::done called with answer :" << r;
    hide();
    emit hideBox();
}

void CommandMessageBox::closeEvent(QCloseEvent *e){
    done(-2);
    /// On close we don't want to destroy the box but just hide it so we ignore the close event
    e->ignore();
    //QMessageBox::closeEvent(e);
}
