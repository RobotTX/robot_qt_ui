#include "doubleclickablebutton.h"
#include <QDebug>
#include <QKeyEvent>

DoubleClickableButton::DoubleClickableButton(const int _id, const QString string, QWidget* parent): QPushButton(string, parent), id(_id)
{
}

void DoubleClickableButton::mouseDoubleClickEvent(QMouseEvent * /* unused */){
    emit doubleClick(id);
}

