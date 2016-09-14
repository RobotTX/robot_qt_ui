#ifndef ROBOTBTNGROUP_H
#define ROBOTBTNGROUP_H

class RobotView;
class QVBoxLayout;

#include <QButtonGroup>
#include <QWidget>

/**
 * @brief The RobotBtnGroup class
 * Class that mix the QButtonGroup class which is not displayable but just a Model
 * with a layout to show the group of buttons
 */
class RobotBtnGroup: public QWidget{
public:
    RobotBtnGroup(const QVector<RobotView*>& vector, QWidget *parent);
    QButtonGroup* getBtnGroup(void) const { return btnGroup; }

public:
    void uncheck();

private:
    QButtonGroup* btnGroup;
    QVBoxLayout* layout;

protected:
    void resizeEvent(QResizeEvent *event);
};

#endif // ROBOTBTNGROUP_H
