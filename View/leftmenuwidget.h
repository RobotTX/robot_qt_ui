#ifndef LEFTMENUWIDGET_H
#define LEFTMENUWIDGET_H

class QMainWindow;
class QVBoxLayout;
class Points;

#include <QWidget>
#include <memory>

/**
 * @brief The LeftMenuWidget class
 * Class which display the first menu with 3 buttons : Robots, Points & MAp
 */
class LeftMenuWidget: public QWidget{
public:
    LeftMenuWidget(QMainWindow* parent, std::shared_ptr<Points> const& _points);

private:
    QVBoxLayout* layout;
    std::shared_ptr<Points> points;

protected:
    void showEvent(QShowEvent *event);
};

#endif // LEFTMENUWIDGET_H
