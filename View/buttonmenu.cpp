#include "buttonmenu.h"
#include <QDebug>
#include <QMouseEvent>


ButtonMenu::ButtonMenu(const QIcon icon,QString text, QWidget* parent): QPushButton(icon,text, parent)
{
     hoverStyle = "QPushButton:after {  content:''; background: grey;  position: absolute;  bottom: 0;  left: 0;  height: 50%; width: 1px;   }""QPushButton:hover{ background-color: grey; border: 1px;}";
     noHoverStyle = "QPushButton{background-position: center center; border: 1px solid;       border-right-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #d3d3d3, stop: 1 transparent);border-left:none; border-top:none; border-bottom:none; position: relative;}";
    this->setStyleSheet(noHoverStyle);
}

 void ButtonMenu::enterEvent(QEvent * ) {
     if(isEnabled())
    this->setStyleSheet(hoverStyle);
 }
 void ButtonMenu::leaveEvent(QEvent * ) {
    this->setStyleSheet(noHoverStyle);
 }


