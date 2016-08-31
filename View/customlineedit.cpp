#include "customlineedit.h"
#include <QDebug>
#include <QShortcut>

CustomLineEdit::CustomLineEdit(QWidget* parent): QLineEdit(parent)
{
}

void CustomLineEdit::focusOutEvent(QFocusEvent* e){
    if(e->reason() == Qt::MouseFocusReason){
        qDebug() << "clicked somewhere";
        emit clickSomewhere(text());
    }
}

void CustomLineEdit::hideEvent(QHideEvent *event){
    Q_UNUSED(event)
    /// enables the edition of a group
    emit enableGroupEdit(true);
    qDebug() << "LineEdit::hideEvent called";
    QWidget::hide();
}

void CustomLineEdit::showEvent(QShowEvent *event){
    Q_UNUSED(event)
    /// disables the edition of a group
    emit enableGroupEdit(false);
    qDebug() << "LineEdit::showEvent called";
    QWidget::show();
}
