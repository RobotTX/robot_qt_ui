#ifndef CUSTOMIZEDLINEEDIT_H
#define CUSTOMIZEDLINEEDIT_H

#include <QLineEdit>
#include <QFocusEvent>

class CustomizedLineEdit: public QLineEdit
{
    Q_OBJECT
public:
    CustomizedLineEdit(QWidget *parent = 0);

public:
    void focusOutEvent(QFocusEvent* e);

signals:
    void clickSomewhere(QString name);
    void pressedEnter();

};

#endif // CUSTOMIZEDLINEEDIT_H
