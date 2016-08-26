#include "custompushbutton.h"
#include "View/stylesettings.h"
#include <QDebug>
#include <QLabel>

CustomPushButton::CustomPushButton(const QIcon &icon, const QString &text, QWidget *parent, const ButtonType type,
        const bool checkable, const bool enable) : QPushButton(icon, text, parent), buttonType(type){
    initialize(checkable, enable);
}

CustomPushButton::CustomPushButton(const QString &text, QWidget *parent, const ButtonType type, const bool checkable,
        const bool enable) : QPushButton(text, parent), buttonType(type){
    initialize(checkable, enable);
}

void CustomPushButton::initialize(const bool checkable, const bool enable){
    setCheckable(checkable);
    setEnabled(enable);
    setFlat(true);

    label = new QLabel("...", this);
    label->hide();

    QString style = "";
    if(text().compare(""))
        style = "padding: 5px 10px 5px 10px; text-align:left;";
    else
        style = "background-position: center center;";



    setStyleSheet("QPushButton {"
                      "color: " + text_color + ";"
                      "border: 1px;"
                      + style +
                  "}"
                  "QPushButton:hover{"
                      "background-color: " + button_hover_color + ";"
                  "}"
                  "QPushButton:checked{"
                      "background-color: " + button_checked_color + ";"
                  "}"
                  "QPushButton:disabled{"
                      "color: grey;"
                  "}"
                  "QLabel {"
                      "color: " + text_color + ";"
                  "}");

    setAutoDefault(true);
}

void CustomPushButton::mouseDoubleClickEvent(QMouseEvent * event){
    Q_UNUSED(event)
    emit doubleClick(text());
}

void CustomPushButton::addStyleSheet(const QString style){
    setStyleSheet(styleSheet() + style);
}

void CustomPushButton::enterEvent(QEvent *event){
    /*if(!text().isEmpty()){
        QFontMetrics fm(font());
        int strWidth = fm.width(text());
        int maxStrWidth = width()-20;
        if(!icon().isNull())
            maxStrWidth -= iconSize().width();

        qDebug() << "CustomPushButton::resizeEvent on" << text() << height();
        if(strWidth >= maxStrWidth){
            //qDebug() << "CustomPushButton::resizeEvent TOO LONG";
            label->move(width()-label->width()+2, height()/3);
            label->show();
        } else {
            //qDebug() << "CustomPushButton::resizeEvent PURRFECT";
            label->hide();
        }
    }*/
    moveLabel();
    QPushButton::enterEvent(event);
}

void CustomPushButton::setText(const QString &str){
    QPushButton::setText(str);
    moveLabel();
}

void CustomPushButton::resizeEvent(QResizeEvent *event){
    if(buttonType == LEFT_MENU){
        QWidget* widget = static_cast<QWidget*>(parent());
        int maxWidth = widget->width()-widget->contentsMargins().right()-widget->contentsMargins().left();
        if(widget->width() > static_cast<QWidget*>(widget->parent())->width()){
            maxWidth = static_cast<QWidget*>(widget->parent())->width()-static_cast<QWidget*>(widget->parent())->contentsMargins().right()-static_cast<QWidget*>(widget->parent())->contentsMargins().left();
        }
        setMaximumWidth(maxWidth);
    }

    QPushButton::resizeEvent(event);
    moveLabel();
}

void CustomPushButton::showEvent(QShowEvent* event){
    QPushButton::showEvent(event);
    moveLabel();
}

void CustomPushButton::moveLabel(){
    if(!text().isEmpty()){
        QFontMetrics fm(font());
        int strWidth = fm.width(text());
        int maxStrWidth = width()-20;
        int iconWidth = 0;
        if(!icon().isNull())
            iconWidth = iconSize().width();
        maxStrWidth -= iconWidth;


        if(strWidth >= maxStrWidth){
            label->move(width()-label->width()+2, height()/3);
            setToolTip(text());
            label->show();
        } else {
            setToolTip("");
            label->hide();
        }
    }
}
