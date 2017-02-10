#ifndef COMMANDCONTROLLER_H
#define COMMANDCONTROLLER_H

class Robot;
class Map;

#include <QObject>
#include "View/Robots/commandmessagebox.h"
#include <QPointer>

/**
 * @brief The CommandController class
 * This class process the command to send to the given robot and wait for an answer
 * while popping a message box to the user
 */
class CommandController : public QObject {
    Q_OBJECT
public:
    CommandController(QWidget *parent);

    /**
     * @brief sendCommand
     * @param robot
     * @param cmd
     * Send the given command to the given robot
     */
    bool sendCommand(QPointer<Robot> robot, QString cmd,
                     QString newRobotName = "", QString groupName = "",
                     QString pathName = "", bool scan = false,
                     int nb = -1, QStringList path = QStringList());

    /**
     * @brief robotDisconnected
     * @param _robotName
     * If the robot disconnect while we send a command,
     * we stop the message box and sendCommand will return false
     */
    void robotDisconnected(QString _robotName);

protected:
    /**
     * @brief openMessageBox
     * @param listCmd
     * Open the message box to tell the user we are processing the command
     */
    void openMessageBox(QStringList listCmd);
    void resetParams();

private slots:
    /**
     * @brief cmdAnswerSlot
     * When we receive a message from a robot
     */
    void cmdAnswerSlot(QString);
    void commandFailed();
    void stopAllCommand();

signals:
    void commandDone(QString cmdName, bool success, QString robotName, QString newRobotName, QString groupName, QString pathName, bool scan, int nb, QStringList path);

private:
    QString robotName;
    /// the message box that's prompted to the user when a command is sent
    CommandMessageBox messageBox;
    QString cmdName;
    bool stop;
    QString newRobotName;
    QString groupName;
    QString pathName;
    bool scan;
    int nb;
    QStringList path;
};

#endif /// COMMANDCONTROLLER_H
