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
#include "View/LeftMenu/topleftmenu.h"
/**
 * @brief The PointsLeftWidget class
 * The purpose of this class is to display a menu on the left of the application relative to the Point objects of the Model
 */

class PointsLeftWidget: public QWidget{
    Q_OBJECT
public:
    PointsLeftWidget(MainWindow* const mainWindow, QSharedPointer<Points> const& _points);

    CustomPushButton* getSaveButton(void) const { return saveButton; }
    CustomPushButton* getCancelButton(void) const { return cancelButton; }
    TopLeftMenu* getActionButtons(void) {return actionButtons;}
    GroupButtonGroup* getGroupButtonGroup(void) const { return groupButtonGroup; }
    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }
    CustomLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }
    QString getLastCheckedId(void) const { return lastCheckedId;}
    bool isCreatingGroup(void) const { return creatingGroup; }

    void setCreatingGroup(const bool create) { creatingGroup = create; }
    void setLastCheckedId(const QString  id) { lastCheckedId = id; }
    void setNameError(const int error) { nameError = error; }

public:
    /**
     * @brief disableButtons
     * disables the buttons when no group is selected
     */
    void disableButtons(void);

    /**
     * @brief resetWidget
     * buttons recreated, pointviews reset to normal and some buttons and lineedit hidden
     */
    void resetWidget(void);


protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent *event);
    void resizeEvent(QResizeEvent *event);

private slots:

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
    /// to reset the path point views on the map
    void resetPathPointViews();
    /// to trigger the same slot as the minus button by using the delete key
    void deleteGroup();
    void updateGroupButtonGroup();
    void resetPointViews();

private:
    CustomLineEdit* groupNameEdit;
    CustomPushButton* saveButton;
    CustomPushButton* cancelButton;
    GroupButtonGroup* groupButtonGroup;
    QLabel* groupNameLabel;
    TopLeftMenu* actionButtons;
    /// to differenciate the behavior of the enter key
    bool creatingGroup;
    QString lastCheckedId;
    int nameError;
};

#endif // POINTSLEFTWIDGET_H

