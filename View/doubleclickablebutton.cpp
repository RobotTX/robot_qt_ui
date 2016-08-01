#include "doubleclickablebutton.h"
#include <QDebug>
#include <QKeyEvent>

DoubleClickableButton::DoubleClickableButton(const QString _id, const QString string, QWidget* parent): QPushButton(string, parent), id(_id)
{
}

void DoubleClickableButton::mouseDoubleClickEvent(QMouseEvent * event){
    Q_UNUSED(event)
    emit doubleClick(id);
}

