#ifndef COMMANDCONTROLLER_H
#define COMMANDCONTROLLER_H

#include <QObject>
#include <QTimer>

class CommandController : public QObject {

    Q_OBJECT

public:

    CommandController(QObject *parent, QString ip, QString robotName);

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

    /**
     * @brief cmdFinished
     * called at the end of <cmdAnswerSlot> to either process the next command or notify the robot model that
     * the command is still being processed
     */
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

    /**
     * @brief updateHome
     * @param ip
     * @param homeX
     * @param homeY
     * signal propagated to the qml side where the home of the robot
     * at ip <ip> is updated
     */
    void updateHome(QString ip, double homeX, double homeY, double homeOri);

    /**
     * @brief updatePath
     * @param ip
     * @param strList
     * propagates through the robot and robots controller until it notifies the qml to
     * update the model
     */
    void updatePath(QString ip, QStringList strList);

    /**
     * @brief stoppedDeletedPath
     * @param ip
     * propagated to the robots controller to reset the path of the robot at ip <ip>
     */
    void stoppedDeletedPath(QString ip);

    /**
     * @brief updatePlayingPath
     * @param ip
     * @param playingPath
     * propagates through the robots controller to notify the qml side to update
     * the status of the robot at ip <ip>
     */
    void updatePlayingPath(QString ip, bool playingPath);

    /**
     * @brief startedScanning
     * @param ip
     * propagates through the robots controller to notify the qml side that the robot
     * at ip <ip> hast started scanning to change the text accordingly
     * adds the map to the window
     */
    void startedScanning(QString ip);

    /**
     * @brief stoppedScanning
     * @param ip
     * propagates through the robots controller to notify the qml side that the robot
     * at ip <ip> hast started scanning to change the text accordingly
     * removes the map from the window
     */
    void stoppedScanning(QString ip);

    /**
     * @brief playedScanning
     * @param ip
     * same as startedScanning
     */
    void playedScanning(QString ip);

    /**
     * @brief pausedScanning
     * @param ip
     * same as stoppedScanning except it does not remove the map
     */
    void pausedScanning(QString ip);

    /**
     * @brief playedExploration
     * @param ip
     * We started to explore
     */
    void playedExploration(QString ip);

    /**
     * @brief pausedExploration
     * @param ip
     * We started to explore
     */
    void pausedExploration(QString ip);

    /**
     * @brief processingCmd
     * @param ip
     * @param waitingForAnswer
     * propagates signal through robots controller to the qml side
     * where the model is updated so that busy indicators can be displayed when relevant
     */
    void processingCmd(QString ip, bool waitingForAnswer);

    /**
     * @brief setMessageTop
     * @param status
     * @param msg
     * Send a signal to set a message on top of the application
     * 0: error
     * 1: warning
     * 2: success
     * 3: info
     */
    void setMessageTop(int status, QString msg);

    /**
     * @brief updateLaser
     * @param ip
     * @param activated
     * Tell the robot model that the robot at ip <ip> as <activated> is lasers
     */
    void updateLaser(QString ip, bool activated);

    /**
     * @brief setLooping
     * @param ip
     * @param looping
     * Tell the robot model that the robot is looping its path
     */
    void setLooping(QString ip, bool looping);

private:
    QString ip;
    /// Only used to set messages on top of the application
    QString robotName;
    QList<QString> cmdQueue;
    QTimer timer;
    bool waitingForAnswer;
};

#endif /// COMMANDCONTROLLER_H
