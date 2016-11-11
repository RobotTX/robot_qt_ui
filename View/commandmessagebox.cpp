#include "commandmessagebox.h"
#include <QTimer>
#include <QPushButton>
#include <QDebug>
#include <QCloseEvent>

CommandMessageBox::CommandMessageBox(QWidget *parent) : QMessageBox(parent){
    /// makes sure the msgbox is deleted automatically when closed
    setAttribute(Qt::WA_DeleteOnClose);
    /// non-modal means it won't block other widgets from receiving slots while the msg box is opened
    setModal(false);
    abortButton = addButton(QMessageBox::Abort);
}

void CommandMessageBox::show(){
    abortButton->hide();

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
    e->ignore();
    //QMessageBox::closeEvent(e);
}
