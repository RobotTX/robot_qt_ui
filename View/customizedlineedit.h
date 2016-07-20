#ifndef CUSTOMIZEDLINEEDIT_H
#define CUSTOMIZEDLINEEDIT_H

#include <QLineEdit>
#include <QFocusEvent>

/**
 * @brief The CustomizedLineEdit class
 * this class provides a customized line edit that emits a particular signal when losing the focus
 */
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
