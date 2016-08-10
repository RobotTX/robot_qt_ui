#ifndef PATHWIDGET_H
#define PATHWIDGET_H

class RobotView;
class QVBoxLayout;
class QPushButton;
class PathPoint;
class QLabel;

#include <QWidget>
#include <memory>

/**
 * @brief The PathWidget class
 * Class which display the path of a robot
 */
class PathWidget: public QWidget{
public:
    PathWidget(QWidget* parent);

    void setPath(QVector<std::shared_ptr<PathPoint>> const path);

    /**
     * @brief clearLayout
     * @param layout
     * Function to clear the layout when modifying it
     */
    void clearLayout(QLayout *layout);

private:
    QWidget* parent;
    QVBoxLayout* layout;
};

#endif // PATHWIDGET_H
