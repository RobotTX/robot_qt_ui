#include "doubleclickablebutton.h"
#include <QDebug>
#include <QKeyEvent>

DoubleClickableButton::DoubleClickableButton(const QString string, QWidget* parent): QPushButton(string, parent)
{
}

void DoubleClickableButton::mouseDoubleClickEvent(QMouseEvent * event){
    Q_UNUSED(event)
    emit doubleClick(text());
}

