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
    PointButtonGroup(const Points& _points, const unsigned int groupIndex, QWidget *parent);
    ~PointButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }

public:
    void deleteButtons(void);
    void setGroup(const Points &_points, const int groupIndex);
    void update(const Points &_points);
    void setCheckable(const bool checkable);
    void uncheck(void);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;

signals:
    void doubleClick(int);
};

#endif // POINTBUTTONGROUP_H
