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
        emit clickSomewhere(text());
    }
}

void CustomLineEdit::hideEvent(QHideEvent *event){
    Q_UNUSED(event)
    qDebug() << "LineEdit::hideEvent called";
    /// enables the edition of a group
    emit enableGroupEdit(true);
}

void CustomLineEdit::showEvent(QShowEvent *event){
    Q_UNUSED(event)
    qDebug() << "LineEdit::showEvent called";
    /// disables the edition of a group
    emit enableGroupEdit(false);
    moveLabel();
}

void CustomLineEdit::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*> (parent());
    int maxWidth = widget->width()-widget->contentsMargins().right()-widget->contentsMargins().left();
    if(widget->width() > static_cast<QWidget*> (widget->parent())->width()){
        maxWidth = static_cast<QWidget*> (widget->parent())->width() - 15 - static_cast<QWidget*>(widget->parent())->contentsMargins().right() - static_cast<QWidget*>(widget->parent())->contentsMargins().left();
    }
    setMaximumWidth(maxWidth);
    QLineEdit::resizeEvent(event);
    moveLabel();
}

void CustomLineEdit::enterEvent(QEvent *event){
    QFontMetrics fm(font());
    qDebug() << "CustomLabel::enterEvent" << text() << size()
             << fm.width(text()) << fm.height();
    QLineEdit::enterEvent(event);
}

void CustomLineEdit::setText(const QString &str){
    QLineEdit::setText(str);
}

void CustomLineEdit::moveLabel(){
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
        setToolTip("");
    }

    /*if(!text().isEmpty()){

        QFontMetrics fm(font());
        int strWidth = fm.width(text());
        int maxStrWidth = width();

        if(strWidth >= maxStrWidth){
            //setStyleSheet("QLineEdit { text-align: right; }");
            //setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

            /// tooltip if disabled
            //setToolTip(text());

        } else {
            //setAlignment(Qt::AlignCenter | Qt::AlignVCenter);
            //setStyleSheet("QLineEdit { text-align: center; }");
            setToolTip("");
        }
    }*/
}
