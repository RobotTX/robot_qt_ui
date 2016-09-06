#ifndef PATHWIDGET_H
#define PATHWIDGET_H

class RobotView;
class QVBoxLayout;
class PathPoint;
class QLabel;

#include <QWidget>
#include <QSharedPointer>

/**
 * @brief The PathWidget class
 * Class which display the path of a robot
 */
class PathWidget: public QWidget{
public:
    PathWidget(QWidget* parent);
    ~PathWidget();

    /**
     * @brief setPath
     * @param path
     * Set the path to display in this widget
     */
    void setPath(QVector<QSharedPointer<PathPoint>> const& path);

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
