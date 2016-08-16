#include "doubleclickablebutton.h"
#include <QDebug>
#include <QKeyEvent>

DoubleClickableButton::DoubleClickableButton(const QString string, QWidget* parent): QPushButton(string, parent)
{
     realName = string ;
     if (realName.length()>7)
     {
         nameToDisplay = string.left(7) + "..";
        setText(nameToDisplay);
     }
     this->setToolTip(realName);
}

void DoubleClickableButton::mouseDoubleClickEvent(QMouseEvent * event){
    Q_UNUSED(event)
    emit doubleClick(realName);
}

