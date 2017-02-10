#ifndef ROBOTBTNGROUP_H
#define ROBOTBTNGROUP_H

class RobotView;
class QVBoxLayout;
class MainWindow;

#include <QButtonGroup>
#include <QWidget>

/**
 * @brief The RobotBtnGroup class
 * Class that mix the QButtonGroup class which is not displayable but just a Model
 * with a layout to show the group of buttons
 */
class RobotBtnGroup: public QWidget {

public:
    RobotBtnGroup(const QVector<QPointer<RobotView>>& vector, MainWindow* mainWindow, QWidget *parent);
    QButtonGroup* getBtnGroup(void) const { return btnGroup; }

public:
    /**
     * @brief uncheck
     * unchecks all buttons of the group
     */
    void uncheck();

private:
    QButtonGroup* btnGroup;
    QVBoxLayout* layout;

protected:
    void resizeEvent(QResizeEvent *event);
};

#endif // ROBOTBTNGROUP_H
