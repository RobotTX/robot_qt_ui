#ifndef COMMANDCONTROLLER_H
#define COMMANDCONTROLLER_H

class Robot;
class Map;

#include <QObject>
#include "View/commandmessagebox.h"
#include <QPointer>

class CommandController : public QObject{
    Q_OBJECT
public:
    CommandController(QWidget *parent);
    void robotWaitForAnswer(QString msg);
    bool sendCommand(QPointer<Robot> robot, QString cmd);
    void sendNewMapToRobot(QPointer<Robot> robot, QString mapId, QSharedPointer<Map> map);
    void robotDisconnected(QString _robotName);

private slots:
    void cmdAnswerSlot(QString);
    void userStopped();

private:
    QPointer<CommandMessageBox> messageBox;
    QString cmdAnswer;
    QString robotName;
};

#endif // COMMANDCONTROLLER_H
