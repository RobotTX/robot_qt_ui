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
    DoubleClickableButton(const QString string, QWidget *parent = 0);
    QString getRealName() {return realName;}
    void setName(QString newName);
private :
    QString realName;
    QString nameToDisplay;
protected:
    void mouseDoubleClickEvent(QMouseEvent *event);

signals:
    void doubleClick(QString);
};

#endif // DOUBLECLICKABLEBUTTON_H
