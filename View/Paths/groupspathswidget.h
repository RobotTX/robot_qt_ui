#ifndef GROUPSPATHSWIDGET_H
#define GROUPSPATHSWIDGET_H

#include <QWidget>
#include "Model/Paths/paths.h"

class CustomLineEdit;
class GroupsPathsButtonGroup;
class TopLeftMenu;
class MainWindow;
class CustomLabel;
class QVBoxLayout;
class QAbstractButton;
class CustomPushButton;
class CustomScrollArea;
class QHBoxLayout;
class PathsController;

class GroupsPathsWidget: public QWidget
{
    Q_OBJECT
public:
    GroupsPathsWidget(PathsController *pathsController);

    GroupsPathsButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    CustomPushButton* getSaveButton(void) const { return saveButton; }
    CustomPushButton* getCancelButton(void) const { return cancelButton; }
    void setCreatingGroup(const bool creating) { creatingGroup = creating; }
    CustomLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }
    CustomLabel* getGroupNameLabel(void) const { return groupNameLabel; }
    QString getLastCheckedButton(void) const { return lastCheckedButton; }

    void setLastCheckedButton(const QString textButton) { lastCheckedButton = textButton; }

public:
    /**
     * @brief initializeActionButtons
     * disables buttons for which a group of path must be selected
     * and resets tooltips of those buttons
     */
    void initializeActionButtons(void);

    /**
     * @brief disableButtons
     * same as initializeActionButtons except for the plus button
     */
    void disableButtons();

    /**
     * @brief updateGroupsPaths
     * called after modifition of one of the group of paths to take into account the modifications
     */
    void updateGroupsPaths(void);

    void uncheck(void);

    /**
     * @brief enableActionButtons
     * enables the action buttons when a group of path is selected
     */
    void enableActionButtons(void);

    /**
     * @brief hideCreationWidgets
     * hides the widgets necessary to the creation of a new group
     * cancel and save buttons, group name label and editLine
     */
    void hideCreationWidgets(void);

    /// sets the widget in the state where u can either click a group or create a new one but nothing else
    /// same state as when u show the widget
    void resetWidget(void);
    void setNameError(const int error) { nameError = error; }

protected:
    void keyPressEvent(QKeyEvent* event);
    void hideEvent(QHideEvent *event);
    void showEvent(QShowEvent *event);
    void resizeEvent(QResizeEvent *event);

signals:
    void newPathGroup(QString);
    void messageCreationGroup(QString, QString);
    void codeEditGroup(int);
    void modifiedGroup(QString);
    /// to delete a group with the delete key
    void deleteGroup();
    void updatePathGroupButtons();

public slots:
    /**
     * @brief cancelCreationGroup
     * cancels the creation of a group, resets the widget (in particular hides the save and cancel buttons)
     */
    void cancelCreationGroup();

private slots:
    /**
     * @brief enableButtons
     * @param button
     * enables buttons to allow the user to perform some actions on the selected group of paths
     */
    void enableButtons(QAbstractButton* button);
    /**
     * @brief newGroupPaths
     * emits a signal caught by the main window to determine if a new group of paths
     * should be created
     */
    void newGroupPaths();

private:
    /// to differenciate the behavior of the enter key
    bool creatingGroup;
    CustomLabel* groupNameLabel;
    CustomLineEdit* groupNameEdit;
    CustomPushButton* saveButton;
    CustomPushButton* cancelButton;
    GroupsPathsButtonGroup* buttonGroup;
    QString lastCheckedButton;
    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;
    int nameError;
};

#endif // GROUPSPATHSWIDGET_H
