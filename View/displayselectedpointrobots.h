#ifndef DISPLAYSELECTEDPOINTROBOTS_H
#define DISPLAYSELECTEDPOINTROBOTS_H

class PointView;
class Robots;
class QButtonGroup;

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
    void removeAllPathButtons();
    QWidget* getHomeWidget(void) const { return homeWidget; }

private slots:
    void robotBtnClicked();
    void pathBtnClicked(QAbstractButton*button);

signals:
    void setSelectedRobotFromPoint(QString);

private:
    QVBoxLayout* layout;
    QWidget* homeWidget;
    QPushButton* robotBtn;

    QWidget* pathWidget;
    QButtonGroup* pathBtnGroup;
    QVBoxLayout* pathBtnLayout;
};

#endif // DISPLAYSELECTEDPOINTROBOTS_H
