#ifndef ROBOTBTNGROUP_H
#define ROBOTBTNGROUP_H

class RobotView;
class QVBoxLayout;
class Robots;

#include <QButtonGroup>
#include <QWidget>

/**
 * @brief The RobotBtnGroup class
 * Class that mix the QButtonGroup class which is not displayable but just a Model
 * with a layout to show the group of buttons
 */
class RobotBtnGroup: public QWidget {

    Q_OBJECT

public:
    RobotBtnGroup(QWidget *parent);
    QButtonGroup* getBtnGroup(void) const { return btnGroup; }

    /**
     * @brief uncheck
     * unchecks all buttons of the group
     */
    void uncheck();

    void updateRobots(QSharedPointer<Robots> robots);

private slots:
    void doubleClickOnRobotSlot(QString robotName);

protected:
    void resizeEvent(QResizeEvent *event);

signals:
    void doubleClickOnRobot(QString);

private:
    QButtonGroup* btnGroup;
    QVBoxLayout* layout;
};

#endif // ROBOTBTNGROUP_H
