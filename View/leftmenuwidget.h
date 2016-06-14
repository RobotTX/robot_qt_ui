#ifndef LEFTMENUWIDGET_H
#define LEFTMENUWIDGET_H

class QMainWindow;
class QVBoxLayout;

#include <QWidget>

/**
 * @brief The LeftMenuWidget class
 * Class which display the first menu with 3 buttons : Robots, Points & MAp
 */
class LeftMenuWidget: public QWidget{
public:
    LeftMenuWidget(QMainWindow* parent);
    ~LeftMenuWidget();

private:
    QVBoxLayout* layout;
};

#endif // LEFTMENUWIDGET_H
