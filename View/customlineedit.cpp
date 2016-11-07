#include "customlineedit.h"
#include <QDebug>
#include <QShortcut>
#include "View/stylesettings.h"

CustomLineEdit::CustomLineEdit(QWidget* parent): QLineEdit(parent){
    initialize();
}

CustomLineEdit::CustomLineEdit(const QString &text, QWidget *parent): QLineEdit(text, parent){
    initialize();
}

void CustomLineEdit::initialize(void){
    /// Some style might be there one day
/*
    setFrame(false);
    setAutoFillBackground(true);
    setStyleSheet(
                "QLineEdit {"
                    "background-color: rgba(255, 0, 0, 0);"
                    "font-weight: bold;"
                    "text-decoration:underline;"
                "}");*/

}

void CustomLineEdit::focusOutEvent(QFocusEvent* e){
    if(e->reason() == Qt::MouseFocusReason){
        qDebug() << "clicked somewhere";
        //emit clickSomewhere(text());
    }
    QLineEdit::focusOutEvent(e);
}

void CustomLineEdit::hideEvent(QHideEvent *event){
    Q_UNUSED(event)
   // qDebug() << "LineEdit::hideEvent called";
    /// enables the edition of a group
    emit enableGroupEdit(true);
    QLineEdit::hideEvent(event);
}

void CustomLineEdit::showEvent(QShowEvent *event){
    Q_UNUSED(event)
    //qDebug() << "LineEdit::showEvent called";
    /// disables the edition of a group
    //emit enableGroupEdit(false);
    updateStyle();
    QLineEdit::showEvent(event);
}

void CustomLineEdit::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width();
    setMaximumWidth(maxWidth);

    QLineEdit::resizeEvent(event);
    updateStyle();
}

void CustomLineEdit::setText(const QString &str){
    QLineEdit::setText(str);
}

void CustomLineEdit::updateStyle(){
    if(isReadOnly()){
        setReadOnly(false);
        home(false);
        setReadOnly(true);
        setStyleSheet("QLineEdit {background-color: " + left_menu_background_color + ";}");
        setFrame(false);
        setToolTip(text());
    } else {
        setStyleSheet("QLineEdit {background-color: #eeeeee;}");
        setFrame(true);
        home(false);
        setToolTip("");
    }
}

void CustomLineEdit::mousePressEvent(QMouseEvent *e){
    qDebug() << "CustomLineEdit::mousePressEvent" << text();
    QLineEdit::mousePressEvent(e);
    if(echoMode() == QLineEdit::Password)
        selectAll();
}
