#include "doubleclickablebutton.h"
#include <QDebug>
#include <QKeyEvent>

DoubleClickableButton::DoubleClickableButton(const QString string, QWidget* parent): QPushButton(string, parent)
{
    setName(string);
}

void DoubleClickableButton::setName(const QString newName)
{
    realName = newName ;
    if (realName.length()>7)
    {
        nameToDisplay = newName.left(7) + "..";
        this->setText(nameToDisplay);
    }
    this->setToolTip(realName);
}


void DoubleClickableButton::mouseDoubleClickEvent(QMouseEvent * event){
    Q_UNUSED(event)
    emit doubleClick(realName);
}

