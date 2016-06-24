#ifndef GROUPBUTTONGROUP_H
#define GROUPBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;

#include <QButtonGroup>
#include <QWidget>
#include <memory>

class GroupButtonGroup: public QWidget
{
public:
    GroupButtonGroup(Points const& _points, QWidget *parent);
    ~GroupButtonGroup();

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }

public:
    void deleteButtons(void);
    void update(const Points& _points);
    void uncheck(void);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
};

#endif // GROUPBUTTONGROUP_H
