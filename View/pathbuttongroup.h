#ifndef PATHBUTTONGROUP_H
#define PATHBUTTONGROUP_H


#include <QWidget>
#include <QButtonGroup>
#include <Model/paths.h>

class QVBoxLayout;

class PathButtonGroup: public QWidget
{
public:
    PathButtonGroup(QWidget *_parent, QSharedPointer<Paths> _paths);
    ~PathButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }

public:
    void setGroupPaths(const QString groupName);
    void deleteButtons(void);
    void uncheck(void);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QSharedPointer<Paths> paths;
    const QSize BUTTON_SIZE;
};

#endif // PATHBUTTONGROUP_H
