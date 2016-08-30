#include "custompushbutton.h"
#include "View/stylesettings.h"
#include <QDebug>
#include <QLabel>


CustomPushButton::CustomPushButton(const QIcon &icon, const QString &text, QWidget *parent, const ButtonType &type, const QString &align,
        const bool &checkable, const bool &enable) : QPushButton(icon, text, parent), buttonType(type){
    initialize(checkable, enable, align);
}

CustomPushButton::CustomPushButton(const QString &text, QWidget *parent, const ButtonType &type, const QString &align, const bool &checkable,
        const bool &enable) : QPushButton(text, parent), buttonType(type){
    initialize(checkable, enable, align);
}

void CustomPushButton::initialize(const bool &checkable, const bool &enable, const QString &align){
    setCheckable(checkable);
    setEnabled(enable);
    setFlat(true);

    label = new QLabel("...", this);
    label->setAttribute(Qt::WA_TranslucentBackground, false);
    label->setStyleSheet(
                "QLabel {"
                    "color: " + text_color + ";"
                    "background-color: " + left_menu_background_color + ";"
                "}");
    label->hide();

    QString style = "";
    if(text().compare(""))
        style = "padding: 5px 10px 5px 10px; text-align:" + align + ";";



    setStyleSheet("QPushButton {"
                      "color: " + text_color + ";"
                      "border: 1px;"
                      + style +
                      "background-position: center center;"
                  "}"
                  "QPushButton:hover {"
                      "background-color: " + button_hover_color + ";"
                  "}"
                  "QPushButton:checked{"
                      "background-color: " + button_checked_color + ";"
                  "}"
                  "QPushButton:disabled{"
                      "color: grey;"
                  "}");

    setAutoDefault(true);
    connect(this, SIGNAL(toggled(bool)), this, SLOT(toggledSlot(bool)));

    switch(buttonType){
        case TOP:
            setMinimumHeight(l_button_height);
            setMaximumHeight(l_button_height);
            setMinimumWidth(l_button_height);
            setMaximumWidth(l_button_height);
        break;
        case BOTTOM:
            setMinimumHeight(m_button_height);
            setMaximumHeight(m_button_height);
        break;
        case LEFT_MENU:
            setMinimumHeight(l_button_height);
            setMaximumHeight(l_button_height);
        break;
        case TOP_LEFT_MENU:
            setMinimumHeight(m_button_height);
            setMaximumHeight(m_button_height);
        break;
        default:
            setMinimumHeight(l_button_height);
            setMaximumHeight(l_button_height);
        break;
    }

    moveLabel();
}

void CustomPushButton::mouseDoubleClickEvent(QMouseEvent * event){
    Q_UNUSED(event)
    emit doubleClick(text());
}

void CustomPushButton::addStyleSheet(const QString style){
    setStyleSheet(styleSheet() + style);
}

void CustomPushButton::setText(const QString &str){
    QPushButton::setText(str);
    moveLabel();
}

void CustomPushButton::toggledSlot(bool checked){
    QString tmpColor = left_menu_background_color;
    if(checked)
        tmpColor = button_checked_color;

    label->setStyleSheet(
                "QLabel {"
                    "color: " + text_color + ";"
                    "background-color: " + tmpColor + ";"
                "}");

    setChecked(checked);
}

void CustomPushButton::resizeEvent(QResizeEvent *event){
    if(buttonType == LEFT_MENU || buttonType == TOP_LEFT_MENU){
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

void CustomPushButton::enterEvent(QEvent *event){
    //qDebug() << "CustomPushButton::enterEvent" << text() << size();
    if(!isChecked())
        label->setStyleSheet(
                    "QLabel {"
                        "color: " + text_color + ";"
                        "background-color: " + button_hover_color + ";"
                    "}");
    QPushButton::enterEvent(event);
}

void CustomPushButton::leaveEvent(QEvent *event){
    if(!isChecked())
        label->setStyleSheet(
                    "QLabel {"
                        "color: " + text_color + ";"
                        "background-color: " + left_menu_background_color + ";"
                    "}");
    QPushButton::leaveEvent(event);
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
            iconWidth = iconSize().width() - 3;
        maxStrWidth -= iconWidth;



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

                    moveTo = QPoint(fm.width(tmpStr) + 10 + iconWidth, height()/3);
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
