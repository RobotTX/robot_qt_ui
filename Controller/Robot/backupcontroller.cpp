#include "backupcontroller.h"
#include "Controller/Robot/backuprobotworker.h"

BackupController::BackupController(const QString ip, const int port, QObject *parent) : QObject(parent), backupThread() {
    backupWorker = QPointer<BackupRobotWorker>(new BackupRobotWorker(ip, port));
    connect(backupWorker, SIGNAL(backupSystemIsDown(QString)), parent, SLOT(backupSystemIsDownSlot(QString)));
    connect(this, SIGNAL(stopBackupWorker()), backupWorker, SLOT(stopWorker()));
    connect(&backupThread, SIGNAL(finished()), backupWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startBackupWorker()), backupWorker, SLOT(connectSocket()));
    connect(this, SIGNAL(reboot()), backupWorker, SLOT(callForReboot()));
    backupWorker->moveToThread(&backupThread);
    backupThread.start();
    emit startBackupWorker();
}

BackupController::~BackupController(){
    emit stopBackupWorker();
    /// stopping the threads might be needed as well
    backupThread.quit();
    backupThread.wait();
}

void BackupController::callForReboot(void){
    emit reboot();
}
