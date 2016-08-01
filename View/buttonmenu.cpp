#include "buttonmenu.h"
#include <QDebug>
#include <QMouseEvent>
#include "colors.h"

ButtonMenu::ButtonMenu(const QIcon icon,QString text, QWidget* parent): QPushButton(icon,text, parent)
{
    QString style = "QPushButton{background-position: center center; border: 1px solid;       border-right-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #d3d3d3, stop: 1 transparent);border-left:none; border-top:none; border-bottom:none; position: relative;";
     hoverStyle = style+"background-color: "+button_hover_color+";}";
     noHoverStyle = style+"}";

    this->setStyleSheet(noHoverStyle);
}


 void ButtonMenu::enterEvent(QEvent * ) {
    if(isEnabled())
    this->setStyleSheet(hoverStyle);
 }
 void ButtonMenu::leaveEvent(QEvent * ) {
    this->setStyleSheet(noHoverStyle);
 }


