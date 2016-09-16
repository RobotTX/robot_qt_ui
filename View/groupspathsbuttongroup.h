#ifndef GROUPS_PATHSBUTTONGROUP_H
#define GROUPS_PATHSBUTTONGROUP_H

#include <QWidget>
#include <QButtonGroup>
#include <Model/paths.h>
#include <QVBoxLayout>

class QVBoxLayout;
class CustomLineEdit;

/**
 * @brief The GroupsPathsButtonGroup class
 * provides a class to display the list of groups of paths
 */
class GroupsPathsButtonGroup: public QWidget {
    Q_OBJECT

public:
    GroupsPathsButtonGroup(QWidget *_parent, const QSharedPointer<Paths>& _paths);

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    CustomLineEdit* getModifyEdit(void) const { return modifyEdit; }
    QVBoxLayout* getLayout(void) const { return layout; }

public:
    /**
     * @brief deleteButtons
     * delete buttons to prepare an update
     */
    void deleteButtons(void);
    /**
     * @brief createButtons
     * creates the buttons (at initialization or to update)
     */
    void createButtons(void);
    /**
     * @brief uncheck
     * unchecks the buttons
     */
    void uncheck(void);
    /**
     * @brief setEnabledGroup
     * @param enable
     * enables or disables the buttons
     */
    void setEnabledGroup(const bool enable);

protected:
    void resizeEvent(QResizeEvent *event);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QSharedPointer<Paths> paths;
    CustomLineEdit* modifyEdit;
    const QSize BUTTON_SIZE;
};

#endif // PATHBUTTONGROUP_H
