#ifndef COMMANDMESSAGEBOX_H
#define COMMANDMESSAGEBOX_H

class QPushButton;

#include <QMessageBox>


class CommandMessageBox : public QMessageBox {
    Q_OBJECT
public:
    CommandMessageBox(QWidget *parent);
    void show();

protected:
    void closeEvent(QCloseEvent *e);

private slots:
    void timerSlot();
    void done(int r);

signals:
    void hideBox();

private:
     QPushButton *abortButton;
};

#endif /// COMMANDMESSAGEBOX_H
