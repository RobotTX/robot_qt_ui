#ifndef POINTBUTTONGROUP_H
#define POINTBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;
class QMouseEvent;

#include <QObject>
#include <QWidget>

class PointButtonGroup: public QWidget
{
    Q_OBJECT
public:
    PointButtonGroup(const Points& _points, const int _groupIndex, QWidget *parent);
    ~PointButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    int getGroupIndex(void) const { return groupIndex; }

public:
    void deleteButtons(void);
    void setGroup(const Points &_points, const int _groupIndex);
    void update(const Points &_points);
    void setCheckable(const bool checkable);
    void uncheck(void);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    int groupIndex;

signals:
    void doubleClick(int);
    void updateConnectionsRequest();
};

#endif // POINTBUTTONGROUP_H
