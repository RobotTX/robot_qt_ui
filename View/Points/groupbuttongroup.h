#ifndef GROUPBUTTONGROUP_H
#define GROUPBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;
class QMouseEvent;

#include "View/Other/customlineedit.h"
#include <QButtonGroup>
#include <QWidget>
#include <QSharedPointer>
#include <QObject>

/**
 * @brief The GroupButtonGroup class
 * provides a widget to display all the groups of points along with points which do not belong to any group
 * also allows a user to perform actions on those objects
 */

class GroupButtonGroup: public QWidget
{
    Q_OBJECT
public:
    GroupButtonGroup(QSharedPointer<Points> const& points, QWidget *parent);

    QButtonGroup* getButtonGroup(void) const { return btnGroup; }
    CustomLineEdit* getModifyEdit(void) const { return modifyEdit; }
    QVBoxLayout* getLayout(void) const { return layout; }
    QString getEditedGroupName(void) const { return editedGroupName; }
    void setEditedGroupName(const QString _editedGroupName) { editedGroupName = _editedGroupName; }
    QAbstractButton* getButtonByName(const QString name) const;
    int getButtonIdByName(const QString name) const;

public:
    /**
     * @brief updateButtons
     * update the buttons for example after a creation, edition or deletions
     */
    void updateButtons(QSharedPointer<Points> const& points);

    /**
     * @brief uncheck
     * unchecks the buttons
     */
    void uncheck(void);

    /**
     * @brief setEnabled
     * @param enable
     * enables or disables the buttons
     */
    void setEnabled(const bool enable);

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent *event);

signals:
    void doubleClick(QString);
    /// emitted when buttons are updated to reestablish the connections
    void updateConnectionsRequest();

private:
    CustomLineEdit* modifyEdit;
    QVBoxLayout* layout;
    QButtonGroup* btnGroup;
    QString editedGroupName;
};

#endif // GROUPBUTTONGROUP_H
