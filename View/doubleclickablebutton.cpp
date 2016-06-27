#include "doubleclickablebutton.h"
#include <QDebug>

DoubleClickableButton::DoubleClickableButton(const int _id, const QString string, QWidget* parent): QPushButton(string, parent), id(_id)
{
}

void DoubleClickableButton::mouseDoubleClickEvent(QMouseEvent *event){
    qDebug() << "double click on button with id " << id;
    emit doubleClick(id);
}
