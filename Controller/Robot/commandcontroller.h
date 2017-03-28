#ifndef COMMANDCONTROLLER_H
#define COMMANDCONTROLLER_H

#include <QObject>

class CommandController : public QObject {
    Q_OBJECT
public:
    CommandController(QObject *parent);
    void sendCommand(const QString cmd);

private slots:
    void cmdAnswerSlot(QString);

signals:
    void sendCommandSignal(QString cmd);
};

#endif // COMMANDCONTROLLER_H
