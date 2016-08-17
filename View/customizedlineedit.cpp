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


void CustomizedLineEdit::hideEvent(QHideEvent *event){
    emit enableGroupEdit(true);
    qDebug() << "LineEdit::hideEvent called";
    QWidget::hide();
}

void CustomizedLineEdit::showEvent(QShowEvent *event){
    emit enableGroupEdit(false);
    qDebug() << "LineEdit::showEvent called";
    QWidget::show();
}
