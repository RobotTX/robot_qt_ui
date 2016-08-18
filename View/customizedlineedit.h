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
    void hideEvent(QHideEvent* event);
    void showEvent(QShowEvent *event);

signals:
    /// emitted when the QLineEdit loses the focus
    void clickSomewhere(QString name);
    void pressedEnter();
    void enableGroupEdit(bool);
};

#endif // CUSTOMIZEDLINEEDIT_H
