#ifndef COMMANDCONTROLLER_H
#define COMMANDCONTROLLER_H

class Robot;
class Map;

#include <QObject>
#include "View/commandmessagebox.h"
#include <QPointer>

/**
 * @brief The CommandController class
 * This class process the command to send to the given robot and wait for an answer
 * while popping a message box to the user
 */
class CommandController : public QObject{
    Q_OBJECT
public:
    CommandController(QWidget *parent);

    /**
     * @brief sendCommand
     * @param robot
     * @param cmd
     * @return if the command succeeded or failed
     * Send the given command to the given robot,
     * wait for an answer and return whether or not the command was succesfully executed
     */
    bool sendCommand(QPointer<Robot> robot, QString cmd);

    /**
     * @brief openMessageBox
     * @param listCmd
     * Open the message box to tell the user we are processing the command
     */
    void openMessageBox(QStringList listCmd);

    /**
     * @brief sendNewMapToRobot
     * @param robot
     * @param mapId
     * @param map
     * Send the given map to the robot
     */
    void sendNewMapToRobot(QPointer<Robot> robot, QSharedPointer<Map> map);

    /**
     * @brief robotDisconnected
     * @param _robotName
     * If the robot disconnect while we send a command,
     * we stop the message box and sendCommand will return false
     */
    void robotDisconnected(QString _robotName);

    /**
     * @brief robotWaitForAnswer
     * @param listCmd
     * @return if the command succeeded or failed
     */
    bool robotWaitForAnswer(QStringList listCmd);

private slots:
    /**
     * @brief cmdAnswerSlot
     * When we receive a message from a robot
     */
    void cmdAnswerSlot(QString);

    /**
     * @brief userStopped
     * If we sent the command but the robot can not process it, the user can press a
     * button to close the message box, and we sendCommand will return false
     */
    void userStopped();

signals:
    void newConnection(QString home_position);

private:
    QPointer<CommandMessageBox> messageBox;
    QString cmdAnswer;
    QString robotName;
    QString cmdName;
};

#endif // COMMANDCONTROLLER_H
