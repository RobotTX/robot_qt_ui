#include "customlabel.h"
#include <QDebug>
#include "View/stylesettings.h"

CustomLabel::CustomLabel(QWidget *parent, bool _title) : QLabel(parent), title(_title){
    initialize();
}

CustomLabel::CustomLabel(const QString &text, QWidget *parent, bool _title) : QLabel(text, parent), title(_title){
    initialize();
}

void CustomLabel::initialize(){

    label = new QLabel("...", this);

    QString style = "";
    if(title){
        QFont tmpFont = font();
        tmpFont.setPointSize(13);
        setFont(tmpFont);
        label->setFont(tmpFont);
    }

    setStyleSheet(
                "QLabel { text-align: left; }");

    label->setAttribute(Qt::WA_TranslucentBackground, false);
    label->setStyleSheet(
                "QLabel {"
                    "color: #222222;"
                    "text-decoration: none;"
                    "padding-right: 5px;"
                    "background-color: " + left_menu_background_color + ";"
                "}"
                "QLabel:disabled {"
                    "color: grey;"
                "}");
    label->hide();
    moveLabel();
}

void CustomLabel::enterEvent(QEvent *event){
    QFontMetrics fm(font());
    qDebug() << "CustomLabel::enterEvent" << text() << size() << label->size()
             << fm.width(text()) << fm.height();
    QLabel::enterEvent(event);
}

void CustomLabel::resizeEvent(QResizeEvent *event){
    //qDebug() << "CustomLabel::resizeEvent" << text() << maximumWidth();
    QWidget* widget = static_cast<QWidget*> (parent());
    int maxWidth = widget->width()-widget->contentsMargins().right()-widget->contentsMargins().left();
    if(widget->width() > static_cast<QWidget*> (widget->parent())->width()){
        maxWidth = static_cast<QWidget*> (widget->parent())->width() - 15 - static_cast<QWidget*>(widget->parent())->contentsMargins().right() - static_cast<QWidget*>(widget->parent())->contentsMargins().left();
    }
    setMaximumWidth(maxWidth);
    QLabel::resizeEvent(event);
    moveLabel();
}

void CustomLabel::moveLabel(){
    if(!text().isEmpty() && !wordWrap()){
        label->setMinimumHeight(height());
        label->setMaximumHeight(height());

        QFontMetrics fm(font());
        QFontMetrics fmLabel(label->font());
        int strWidth = fm.width(text());
        int maxStrWidth = width() - fmLabel.width(label->text()) - 5;

        if(strWidth >= maxStrWidth){
            QPoint moveTo = QPoint(0, 0);
            QString str = text();
            QString tmpStr = "";

            for(int i = 0; i < str.size(); i++){
                tmpStr += str.at(i);
                if(fm.width(tmpStr) >= maxStrWidth){
                    tmpStr.remove(tmpStr.size()-1, 1);
                    if(!tmpStr.isEmpty() && tmpStr.at(tmpStr.size()-1) == ' ')
                        tmpStr.remove(tmpStr.size()-1, 1);

                    moveTo = QPoint(fm.width(tmpStr), 0);

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
