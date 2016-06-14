#ifndef GROUPBUTTONGROUP_H
#define GROUPBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;

#include <QButtonGroup>
#include <QWidget>

class GroupButtonGroup: public QWidget
{
public:
    GroupButtonGroup(const Points& _points);
    ~GroupButtonGroup();

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }

public:
    void deleteButtons(void);
    void update(const Points& _points);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;

};

#endif // GROUPBUTTONGROUP_H
