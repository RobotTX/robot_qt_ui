#ifndef LEFTMENUWIDGET_H
#define LEFTMENUWIDGET_H

class QMainWindow;
class QVBoxLayout;
class Points;
class CustomPushButton;

#include <QWidget>
#include <QSharedPointer>

/**
 * @brief The LeftMenuWidget class
 * Class which display the first menu with 3 buttons : Robots, Points & MAp
 */
class LeftMenuWidget: public QWidget{
    Q_OBJECT
public:
    LeftMenuWidget(QWidget* parent, QSharedPointer<Points> const& _points);
    CustomPushButton* getRobotBtn(void) const {return robotBtn; }
    CustomPushButton* getPointBtn(void) const {return pointBtn; }
    CustomPushButton* getMapBtn(void) const {return mapBtn; }
    CustomPushButton* getPathBtn(void) const {return pathBtn; }

private:
    QVBoxLayout* layout;
    QSharedPointer<Points> points;
    CustomPushButton* robotBtn;
    CustomPushButton* pointBtn;
    CustomPushButton* mapBtn;
    CustomPushButton* pathBtn;

protected:
    void showEvent(QShowEvent *event);

signals:
    /// when showing this widget the application resets the colors of the path points on the map
    void resetPathPointViews();
};

#endif // LEFTMENUWIDGET_H
