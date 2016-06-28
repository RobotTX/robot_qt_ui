#ifndef DOUBLECLICKABLEBUTTON_H
#define DOUBLECLICKABLEBUTTON_H

class QMouseEvent;

#include <QPushButton>

/**
 * @brief The DoubleClickableButton class
 * this class provides a customized button that handles double clicks
 */
class DoubleClickableButton: public QPushButton
{
    Q_OBJECT
public:
    DoubleClickableButton(const int _id, const QString string, QWidget *parent = 0);

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);

signals:
    void doubleClick(int);

private:
    int id;
};

#endif // DOUBLECLICKABLEBUTTON_H
