#ifndef COMMANDCONTROLLER_H
#define COMMANDCONTROLLER_H

#include <QObject>

class CommandController : public QObject {
    Q_OBJECT
public:
    CommandController(QObject *parent, QString ip);
    void sendCommand(const QString cmd);

private slots:
    void cmdAnswerSlot(QString);

signals:
    void sendCommandSignal(QString cmd);
    void updateName(QString ip, QString newName);
    void updateHome(QString ip, QString homeName, float homeX, float homeY);
    void updatePath(QString ip, QStringList strList);
    void stoppedDeletedPath(QString ip);

private:
    QString ip;
};

#endif // COMMANDCONTROLLER_H
