#ifndef LEFTMENUWIDGET_H
#define LEFTMENUWIDGET_H

class QMainWindow;
class QVBoxLayout;
class Points;

#include <QWidget>
#include <QSharedPointer>

/**
 * @brief The LeftMenuWidget class
 * Class which display the first menu with 3 buttons : Robots, Points & MAp
 */
class LeftMenuWidget: public QWidget{
    Q_OBJECT
public:
    LeftMenuWidget(QMainWindow* parent, QSharedPointer<Points> const& _points);

private:
    QVBoxLayout* layout;
    QSharedPointer<Points> points;

protected:
    void showEvent(QShowEvent *event);

signals:
    /// when showing this widget the application resets the colors of the path points on the map
    void resetPathPointViews();
};

#endif // LEFTMENUWIDGET_H
