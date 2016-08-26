#include "customlabel.h"
#include <QDebug>

CustomLabel::CustomLabel(QWidget *parent):QLabel(parent){

}

CustomLabel::CustomLabel(const QString &text, QWidget *parent):QLabel(text, parent){

}

void CustomLabel::resizeEvent(QResizeEvent *event){
    //qDebug() << "CustomLabel::resizeEvent" << text() << maximumWidth();
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width()-widget->contentsMargins().right()-widget->contentsMargins().left();
    if(widget->width() > static_cast<QWidget*>(widget->parent())->width()){
        maxWidth = static_cast<QWidget*>(widget->parent())->width() - 15 - static_cast<QWidget*>(widget->parent())->contentsMargins().right() - static_cast<QWidget*>(widget->parent())->contentsMargins().left();
    }
    setMaximumWidth(maxWidth);

    QLabel::resizeEvent(event);
}
