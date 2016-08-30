#include "customlabel.h"
#include <QDebug>
#include "View/stylesettings.h"

CustomLabel::CustomLabel(QWidget *parent):QLabel(parent){
    initialize();
}

CustomLabel::CustomLabel(const QString &text, QWidget *parent):QLabel(text, parent){
    initialize();
}

void CustomLabel::initialize(){
    label = new QLabel("...", this);
    label->setAttribute(Qt::WA_TranslucentBackground, false);
    label->setStyleSheet(
                "QLabel {"
                    "color: #222222;"
                    "text-decoration: none;"
                    "background-color: " + left_menu_background_color + ";"
                "}"
                "QLabel:disabled {"
                    "color: grey;"
                "}");
    label->hide();
    setMinimumHeight(s_button_height);
    setMaximumHeight(s_button_height);

    moveLabel();
}

void CustomLabel::enterEvent(QEvent *event){
    qDebug() << "CustomLabel::enterEvent" << text() << size();
    QLabel::enterEvent(event);
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
    moveLabel();
}

void CustomLabel::moveLabel(){
    if(!text().isEmpty() && !wordWrap()){
        QFontMetrics fm(font());
        int strWidth = fm.width(text());
        int maxStrWidth = width()-20;

        if(strWidth >= maxStrWidth){
            QPoint moveTo = QPoint(0, 0);
            QString str = text();
            QString tmpStr = "";

            for(int i = 0; i < str.size(); i++){
                tmpStr += str.at(i);
                if(fm.width(tmpStr) >= maxStrWidth){
                    tmpStr.remove(tmpStr.size()-1, 1);
                    if(tmpStr.at(tmpStr.size()-1) == ' ')
                        tmpStr.remove(tmpStr.size()-1, 1);

                    moveTo = QPoint(fm.width(tmpStr) + 10, height()/3);
                    if(height() < l_button_height)
                        moveTo.setY(moveTo.y() - 2);

                    break;
                }
            }

            label->move(moveTo);
            setToolTip(text());
            label->show();
        } else {
            setToolTip("");
            label->hide();
        }
    }
}
