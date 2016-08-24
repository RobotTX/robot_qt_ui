#include "custompushbutton.h"
#include "View/colors.h"

CustomPushButton::CustomPushButton(const QIcon &icon, const QString &text, QWidget *parent,
        const bool checkable, const bool enable) : QPushButton(icon, text, parent){
    initialize(checkable, enable);
}

CustomPushButton::CustomPushButton(const QString &text, QWidget *parent, const bool checkable,
        const bool enable) : QPushButton(text, parent){
    initialize(checkable, enable);
}

void CustomPushButton::initialize(const bool checkable, const bool enable){
    setCheckable(checkable);
    setEnabled(enable);
    setFlat(true);

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
                  "}");

    setAutoDefault(true);
}

void CustomPushButton::mouseDoubleClickEvent(QMouseEvent * event){
    Q_UNUSED(event)
    emit doubleClick(text());
}

void CustomPushButton::addStyleSheet(QString style){
    setStyleSheet(styleSheet() + style);
}

