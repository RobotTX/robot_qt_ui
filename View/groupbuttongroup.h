#ifndef GROUPBUTTONGROUP_H
#define GROUPBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;
class QMouseEvent;

#include "View/customlineedit.h"
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
    GroupButtonGroup(QSharedPointer<Points> const& _points, QWidget *_parent);

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    CustomLineEdit* getModifyEdit(void) const { return modifyEdit; }
    QVBoxLayout* getLayout(void) const { return layout; }
    QString getEditedGroupName(void) const { return editedGroupName; }
    int getEditedGroupId(void) const;
    void setEditedGroupName(const QString _editedGroupName) { editedGroupName = _editedGroupName; }
    QAbstractButton* getButtonByName(const QString name) const;
    int getButtonIdByName(const QString name) const;

    /// removes useless spaces
    QString formatName(const QString name) const;
    void hideModifyEdit(void) { modifyEdit->hide(); }

public:
    /**
     * @brief deleteButtons
     * delete the buttons so they can be recreated (to update our widget)
     */
    void deleteButtons(void);
    /**
     * @brief deleteButton
     * update the buttons for example after a creation, edition or deletions
     */
    void updateButtons();
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
    /// emitted when buttons are updated to reestablish the connection with the line edit to modify a group
    void modifyEditReconnection();
    /// emitted upon edition of a group's name to signify the mainWindow that the name is valid or not
    void codeEditGroup(int);

public slots:

    int checkEditGroupName(QString name);

private:
    CustomLineEdit* modifyEdit;
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QWidget* parentWidget;
    QString editedGroupName;
    QSharedPointer<Points> points;
};

#endif // GROUPBUTTONGROUP_H
