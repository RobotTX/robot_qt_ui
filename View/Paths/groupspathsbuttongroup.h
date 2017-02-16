#ifndef GROUPS_PATHSBUTTONGROUP_H
#define GROUPS_PATHSBUTTONGROUP_H

#include <QWidget>
#include <QButtonGroup>
#include <QVBoxLayout>
#include "Model/Paths/paths.h"

class QVBoxLayout;
class CustomLineEdit;

/**
 * @brief The GroupsPathsButtonGroup class
 * provides a class to display the list of groups of paths
 */
class GroupsPathsButtonGroup: public QWidget {
    Q_OBJECT

public:
    GroupsPathsButtonGroup(QWidget *_parent);

    QButtonGroup* getButtonGroup(void) const { return btnGroup; }
    CustomLineEdit* getModifyEdit(void) const { return modifyEdit; }
    QVBoxLayout* getLayout(void) const { return layout; }

public:
    /**
     * @brief updateButtons
     * updateButtons the buttons (at initialization or to update)
     */
    void updateButtons(QSharedPointer<Paths> const& paths);

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

signals:
    /// emitted when buttons are updated to reestablish the connections
    void updateConnectionsRequest();

private:
    QVBoxLayout* layout;
    QButtonGroup* btnGroup;
    CustomLineEdit* modifyEdit;
};

#endif // PATHBUTTONGROUP_H
