#ifndef COMMANDMESSAGEBOX_H
#define COMMANDMESSAGEBOX_H

class QPushButton;
class QTimer;

#include <QMessageBox>


class CommandMessageBox : public QMessageBox {
    Q_OBJECT
public:
    CommandMessageBox(QWidget *parent = 0);
    void show();

protected:
    void closeEvent(QCloseEvent *e);

public slots:
    void timerSlot();
    void done(int r);

signals:
    void hideBox();

private:
     QPushButton *abortButton;
     QTimer* timer;
};

#endif /// COMMANDMESSAGEBOX_H
