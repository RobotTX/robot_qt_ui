#include "custompushbutton.h"
#include "View/stylesettings.h"
#include <QDebug>
#include <QLabel>
#include <QResizeEvent>


CustomPushButton::CustomPushButton(const QIcon& icon, const QString text, QWidget *parent, const bool _customTooltipEnable, const ButtonType type, const QString align,
        const bool checkable, const bool enable) :
    QPushButton(icon, text, parent), buttonType(type), customTooltipEnable(_customTooltipEnable)
{
    initialize(checkable, enable, align);
}

CustomPushButton::CustomPushButton(const QString text, QWidget *parent, const bool _customTooltipEnable, const ButtonType type, const QString align, const bool checkable,
        const bool enable) : QPushButton(text, parent), buttonType(type), customTooltipEnable(_customTooltipEnable)
{
    initialize(checkable, enable, align);
}

void CustomPushButton::initialize(const bool checkable, const bool enable, const QString align){

    /// Set the style of the label "..." which is used when a text is too long for the button to display "text..." without modifying the text
    label = new QLabel("...", this);
    label->setAttribute(Qt::WA_TranslucentBackground, false);
    label->setStyleSheet(
                "QLabel {"
                    "color: " + text_color + ";"
                    "background-color: " + left_menu_background_color + ";"
                "}"
                "QLabel:disabled {"
                    "color: grey;"
                "}");

    label->hide();

    /// Style of the button
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
                  "QPushButton:checked {"
                      "background-color: " + button_checked_color + ";"
                  "}"
                  "QPushButton:disabled{"
                      "color: grey;"
                  "}");

    setAutoDefault(true);
    connect(this, SIGNAL(toggled(bool)), this, SLOT(toggledSlot(bool)));

    /// Set the size of the button depending of its type
    switch(buttonType){
        case TOP:
            setMinimumHeight(l_button_height);
            setMaximumHeight(l_button_height);
            setMinimumWidth(l_button_height);
            setMaximumWidth(l_button_height);
            label->setMinimumHeight(l_button_height);
            label->setMaximumHeight(l_button_height);
        break;
        case BOTTOM:
            setMinimumHeight(m_button_height);
            setMaximumHeight(m_button_height);
            label->setMinimumHeight(m_button_height);
            label->setMaximumHeight(m_button_height);
        break;
        case LEFT_MENU:
            setMinimumHeight(l_button_height);
            setMaximumHeight(l_button_height);
            label->setMinimumHeight(l_button_height);
            label->setMaximumHeight(l_button_height);
        break;
        case TOP_LEFT_MENU:
            setMinimumHeight(m_button_height);
            setMaximumHeight(m_button_height);
            label->setMinimumHeight(m_button_height);
            label->setMaximumHeight(m_button_height);
        break;
        default:
            setMinimumHeight(l_button_height);
            setMaximumHeight(l_button_height);
            label->setMinimumHeight(l_button_height);
            label->setMaximumHeight(l_button_height);
        break;
    }

    setCheckable(checkable);
    setEnabled(enable);
    setFlat(true);

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
    //qDebug() << "CustomPushButton::setText" << str << text();
    moveLabel();
}

void CustomPushButton::toggledSlot(bool checked){
    /// When toggled, we also need to change the style of the label
    QString tmpColor = left_menu_background_color;
    if(checked)
        tmpColor = button_checked_color;
    else
        if(hasFocus())
            tmpColor = button_hover_color;

    label->setStyleSheet(
                "QLabel {"
                    "color: " + text_color + ";"
                    "background-color: " + tmpColor + ";"
                "}"
                "QLabel:disabled {"
                    "color: grey;"
                "}");

    setChecked(checked);
}

void CustomPushButton::setEnabled(bool checked){
    label->setEnabled(checked);
    QPushButton::setEnabled(checked);
}

void CustomPushButton::resizeEvent(QResizeEvent *event){
    if(buttonType == LEFT_MENU || buttonType == TOP_LEFT_MENU){
        QWidget* widget = static_cast<QWidget*>(parent());
        int maxWidth = widget->width();
        setMaximumWidth(maxWidth);
    }

    QPushButton::resizeEvent(event);
    moveLabel();
}

void CustomPushButton::enterEvent(QEvent *event){
    if(!isChecked() && isEnabled())
        label->setStyleSheet(
                    "QLabel {"
                        "color: " + text_color + ";"
                        "background-color: " + button_hover_color + ";"
                    "}");
    QPushButton::enterEvent(event);
}

void CustomPushButton::leaveEvent(QEvent *event){
    if(!isChecked() && isEnabled())
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

        /// If the text is too long for the button we display "..." at the right position
        if(maxStrWidth > 0 && strWidth >= maxStrWidth){
            QPoint moveTo = QPoint(0, 0);
            QString str = text();
            QString tmpStr = "";

            for(int i = 0; i < str.size(); i++){
                tmpStr += str.at(i);
                if(fm.width(tmpStr) >= maxStrWidth){
                    tmpStr.remove(tmpStr.size()-1, 1);
                    if(tmpStr.at(tmpStr.size()-1) == ' ')
                        tmpStr.remove(tmpStr.size()-1, 1);

                    moveTo = QPoint(fm.width(tmpStr) + 10 + iconWidth, 0);

                    break;
                }
            }

            label->move(moveTo);
            setToolTip(text());
            label->show();

        } else {
            if(!customTooltipEnable)
                setToolTip("");

            label->hide();
        }
    }
}
