#ifndef GROUPS_PATHSBUTTONGROUP_H
#define GROUPS_PATHSBUTTONGROUP_H

#include <QWidget>
#include <QButtonGroup>
#include <Model/paths.h>
#include <QVBoxLayout>

class QVBoxLayout;
class CustomizedLineEdit;

class GroupsPathsButtonGroup: public QWidget {
    Q_OBJECT

public:
    GroupsPathsButtonGroup(QWidget *_parent, QSharedPointer<Paths> _paths);

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    CustomizedLineEdit* getModifyEdit(void) const { return modifyEdit; }
    QVBoxLayout* getLayout(void) const { return layout; }

    void deleteButtons(void);
    void setEnabledGroup(const bool enable);

public:
    void createButtons(void);
    void uncheck(void);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QSize BUTTON_SIZE;
    QSharedPointer<Paths> paths;
    CustomizedLineEdit* modifyEdit;
};

#endif // PATHBUTTONGROUP_H
