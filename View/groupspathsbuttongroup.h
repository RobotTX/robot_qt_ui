#ifndef GROUPS_PATHSBUTTONGROUP_H
#define GROUPS_PATHSBUTTONGROUP_H

#include <QWidget>
#include <QButtonGroup>
#include <Model/paths.h>
#include <QVBoxLayout>

class QVBoxLayout;
class CustomLineEdit;

class GroupsPathsButtonGroup: public QWidget {
    Q_OBJECT

public:
    GroupsPathsButtonGroup(QWidget *_parent, const QSharedPointer<Paths>& _paths);

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    CustomLineEdit* getModifyEdit(void) const { return modifyEdit; }
    QVBoxLayout* getLayout(void) const { return layout; }

    void deleteButtons(void);
    void setEnabledGroup(const bool enable);

public:
    void createButtons(void);
    void uncheck(void);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QSharedPointer<Paths> paths;
    CustomLineEdit* modifyEdit;
    const QSize BUTTON_SIZE;
};

#endif // PATHBUTTONGROUP_H
