#ifndef GROUPBUTTONGROUP_H
#define GROUPBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;
class QMouseEvent;

#include <QButtonGroup>
#include <QWidget>
#include <memory>
#include <QObject>

class GroupButtonGroup: public QWidget
{
    Q_OBJECT
public:
    GroupButtonGroup(Points const& _points, QWidget *parent);
    ~GroupButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }

public:
    void deleteButtons(void);
    void update(const Points& _points);
    void uncheck(void);

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;

signals:
    void doubleClick(int);
};

#endif // GROUPBUTTONGROUP_H
