#ifndef BUTTONMENU_H
#define BUTTONMENU_H

#include <QPushButton>

class ButtonMenu: public QPushButton
{
    Q_OBJECT
public:
    ButtonMenu( const QIcon icon,QString text, QWidget *parent = 0);

protected:
    void enterEvent(QEvent * e) ;
    void leaveEvent(QEvent * e) ;

private:
    QString noHoverStyle;
    QString hoverStyle;
};

#endif // BUTTONMENU_H

