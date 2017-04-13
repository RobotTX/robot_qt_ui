#ifndef COMMANDCONTROLLER_H
#define COMMANDCONTROLLER_H

#include <QObject>
#include <QTimer>

class CommandController : public QObject {
    Q_OBJECT
public:
    CommandController(QObject *parent, QString ip);

    /**
     * @brief sendCommand
     * @param cmd
     * Send the command <cmd> to the robot
     */
    void sendCommand(const QString cmd);

private slots:
    /**
     * @brief cmdAnswerSlot
     * Receive the answer of the command we sent to the robot
     */
    void cmdAnswerSlot(QString);
    void cmdFinished(void);

signals:
    /**
     * @brief sendCommandSignal
     * @param cmd
     * Send the command <cmd> to the robot
     */
    void sendCommandSignal(QString cmd);

    /// Signals sent to update the model and view when we've executed a command
    void updateName(QString ip, QString newName);
    void updateHome(QString ip, QString homeName, float homeX, float homeY);
    void updatePath(QString ip, QStringList strList);
    void stoppedDeletedPath(QString ip);
    void updatePlayingPath(QString ip, bool playingPath);
    void startedScanning(QString ip);
    void stoppedScanning(QString ip);
    void playedScanning(QString ip);
    void pausedScanning(QString ip);
    void processingCmd(QString ip, bool waitingForAnswer);

private:
    QString ip;
    QList<QString> cmdQueue;
    QTimer timer;
    bool waitingForAnswer;
};

#endif // COMMANDCONTROLLER_H
