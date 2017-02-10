#ifndef TELEOPWIDGET_H
#define TELEOPWIDGET_H

class QButtonGroup;

#include <QWidget>

class TeleopWidget : public QWidget {
    Q_OBJECT
public:
    TeleopWidget(QWidget *parent);
    QButtonGroup* getBtnGroup(void) const { return btnGroup; }

private:
    QButtonGroup* btnGroup;
};

#endif // TELEOPWIDGET_H
