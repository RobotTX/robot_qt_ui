#ifndef DISPLAYSELECTEDPOINTROBOTS_H
#define DISPLAYSELECTEDPOINTROBOTS_H

class PointView;
class Robots;

#include <QWidget>
#include <QVBoxLayout>
#include <memory>
#include <QPushButton>
#include <QObject>

class DisplaySelectedPointRobots: public QWidget {
    Q_OBJECT
public:
    DisplaySelectedPointRobots(QWidget* parent);
    void setRobotsWidget(PointView* pointView, std::shared_ptr<Robots> robots, const QString robotName = "");
    void removeAllItems(QLayout *_layout);
    QWidget* getHomeWidget(void) const { return homeWidget; }
    QPushButton* getRobotButton(void) const { return robotBtn; }

private slots:
    void robotBtnClicked();

signals:
    void setSelectedRobotFromPoint(QString);

private:
    QVBoxLayout* layout;
    QWidget* homeWidget;
    QPushButton* robotBtn;
};

#endif // DISPLAYSELECTEDPOINTROBOTS_H
