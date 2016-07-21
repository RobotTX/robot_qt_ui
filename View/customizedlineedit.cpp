#include "customizedlineedit.h"
#include <QDebug>
#include <QShortcut>

CustomizedLineEdit::CustomizedLineEdit(QWidget* parent): QLineEdit(parent)
{
}

void CustomizedLineEdit::focusOutEvent(QFocusEvent* e){
    if(e->reason() == Qt::MouseFocusReason){
        qDebug() << "clicked somewhere";
        emit clickSomewhere(text());
    }
}

