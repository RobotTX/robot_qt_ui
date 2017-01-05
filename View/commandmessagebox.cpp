#include "commandmessagebox.h"
#include <QTimer>
#include <QPushButton>
#include <QDebug>
#include <QCloseEvent>

CommandMessageBox::CommandMessageBox(QWidget *parent) : QMessageBox(parent){
    /// non-modal means it won't block other widgets from receiving slots while the msg box is opened
    setModal(false);
    abortButton = addButton(QMessageBox::Abort);
    //setAttribute(Qt::WA_DeleteOnClose);
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
    emit hideBox();
    //QMessageBox::done(r);
}

void CommandMessageBox::closeEvent(QCloseEvent *e){
    qDebug() << "CommandMessageBox::closeEvent called" << e;
    //QMessageBox::closeEvent(e);
    e->ignore();
    show();
    hide();
}
