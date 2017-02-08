#ifndef POINTSLEFTWIDGET_H
#define POINTSLEFTWIDGET_H

class CustomScrollArea;
class PointButtonGroup;
class GroupButtonGroup;
class Points;
class QMainWindow;
class QVBoxLayout;
class CustomPushButton;
class QLabel;
class CustomLineEdit;
class QHBoxLayout;
class GroupEditWindow;
class Points;
class QAbstractButton;
class CustomLineEdit;
class MainWindow;
class PointsController;

#include <QWidget>
#include <QSharedPointer>
#include "topleftmenu.h"
/**
 * @brief The PointsLeftWidget class
 * The purpose of this class is to display a menu on the left of the application relative to the Point objects of the Model
 */

class PointsLeftWidget: public QWidget{
    Q_OBJECT
public:
    PointsLeftWidget(MainWindow* const mainWindow, QSharedPointer<Points> const& _points, bool _groupDisplayed = true);

    bool getGroupDisplayed(void) const { return groupDisplayed; }
    void setGroupDisplayed(const bool _groupDisplayed) { groupDisplayed = _groupDisplayed; }
    QSharedPointer<Points> getPoints(void) const { return points; }
    CustomPushButton* getSaveButton(void) const { return saveButton; }
    CustomPushButton* getCancelButton(void) const { return cancelButton; }
    TopLeftMenu* getActionButtons(void) {return actionButtons;}
    GroupButtonGroup* getGroupButtonGroup(void) const { return groupButtonGroup; }
    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }
    CustomLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }
    CustomScrollArea* getScrollArea(void) const { return scrollArea; }
    QString getLastCheckedId() const { return lastCheckedId;}

    void setCreatingGroup(const bool create) { creatingGroup = create; }
    void setLastCheckedId(const QString  id) { lastCheckedId = id; }

public:
    /**
     * @brief disableButtons
     * disables the buttons when no group is selected
     */
    void disableButtons(void);
    /**
     * @brief updateGroupButtonGroup
     * to update the buttons when some modification occured (group edited or added)
     */
    void updateGroupButtonGroup();
    /**
     * @brief resetWidget
     * buttons recreated, pointviews reset to normal and some buttons and lineedit hidden
     */
    void resetWidget(void);

private:
    /**
     * @brief formatName
     * @param name
     * @return formated name
     * format a name to remove extra spaces
     */
    QString formatName(const QString name) const;

protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent *event);
    void resizeEvent(QResizeEvent *event);

public slots:
    /**
      * @brief checkGroupName
      * @param name
      * @return int (error code)
      * checks whether the name is valid (not empty and not taken)
      */
     int checkGroupName(QString name);

private slots:
     /**
     * @brief enableButtons
     * @param button
     * enables buttons when a group is selected
     */
    void enableButtons(QString button);
    void enableButtons(QAbstractButton* button);
    /**
     * @brief cancelCreationGroup
     * called when the user clicks somewhere at random in the window (during creation of a group)
     */
    void cancelCreationGroup();
    /**
     * @brief emitNewGroupSignal
     * emits a signal to create a new group
     */
    void emitNewGroupSignal();
    /**
     * @brief modifyGroupAfterClick
     * @param name
     * to modify a group when a user clicks somewhere else in the window (during edition of a group)
     */
    void modifyGroupAfterClick(QString name);
    /**
     * @brief reconnectModifyEdit
     * to reconnect the modify edit after a group has been edited
     */
    void reconnectModifyEdit();
    /**
     * @brief sendMessageEditGroup
     * @param code
     * so that an appropriate message can be displayed while a group is being edited
     */
    void sendMessageEditGroup(int code);

signals:
    /// emitted when a new group is created
    void newGroup(QString name);
    /// emitted when a group is modified
    void modifiedGroup(QString name);
    /// emitted when a group is modified after a click (could be a random click on the map)
    void modifiedGroupAfterClick(QString name);
    /// emitted to enable the return button and the close button
    void enableReturn();
    /// to display an appropriate message regarding the creation of a group
    void messageCreationGroup(QString, QString);
    /// to display an appropriate message regarding the creation of a point
    void messageCreationPoint();
    /// to reset the path point views on the map
    void resetPathPointViews();
    /// to trigger the same slot as the minus button by using the delete key
    void deleteGroup();

private:

    QVBoxLayout* layout;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;
    QHBoxLayout* creationLayout;

    GroupButtonGroup* groupButtonGroup;

    CustomPushButton* saveButton;
    CustomPushButton* cancelButton;

    QLabel* groupNameLabel;
    CustomLineEdit* groupNameEdit;

    CustomScrollArea* scrollArea;

    TopLeftMenu* actionButtons;

    /// true if the groups are displayed, false if the points are displayed
    /// this way we can implement two different behavior for the same button minus
    bool groupDisplayed;
    QSharedPointer<Points> points;
    /// to differenciate the behavior of the enter key
    bool creatingGroup;
    QString lastCheckedId;
};

#endif // POINTSLEFTWIDGET_H

