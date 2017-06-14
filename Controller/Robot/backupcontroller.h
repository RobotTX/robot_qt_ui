#ifndef BACKUPCONTROLLER_H
#define BACKUPCONTROLLER_H

class BackupRobotWorker;

#include <QObject>
#include <QThread>
#include <QPointer>

/**
 * @brief The BackupController class
 * Client to call a service from server running on the robot
 * to restart the packages of the robot
 */

class BackupController : public QObject {

    Q_OBJECT

public:
    BackupController(const QString ip, const int port, QObject *parent = Q_NULLPTR);
    ~BackupController();

    void callForReboot(void);

signals:
    void reboot();
    void startBackupWorker();
    void stopBackupWorker();

private:
    QThread backupThread;
    QPointer<BackupRobotWorker> backupWorker;
};

#endif /// BACKUPCONTROLLER_H
