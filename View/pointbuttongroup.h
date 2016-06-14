#ifndef POINTBUTTONGROUP_H
#define POINTBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;

#include <QWidget>

class PointButtonGroup: public QWidget
{
public:
    PointButtonGroup(const Points& _points, const unsigned int groupIndex);
    ~PointButtonGroup();

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }


public:
    void deleteButtons(void);
    void setGroup(const Points &_points, const int groupIndex);
    void update(const Points &_points);
private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
};

#endif // POINTBUTTONGROUP_H
