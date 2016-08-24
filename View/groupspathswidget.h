#ifndef GROUPSPATHSWIDGET_H
#define GROUPSPATHSWIDGET_H

#include <QWidget>
#include "Model/paths.h"

class CustomizedLineEdit;
class GroupsPathsButtonGroup;
class PathButtonGroup;
class TopLeftMenu;
class MainWindow;
class QLabel;
class QVBoxLayout;
class QAbstractButton;
class CustomPushButton;
class CustomScrollArea;
class QHBoxLayout;

class GroupsPathsWidget: public QWidget
{
    Q_OBJECT
public:
    GroupsPathsWidget(MainWindow* _parent, const QSharedPointer<Paths> &_paths);

    PathButtonGroup* getPathButtonGroup(void) const { return pathButtonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    CustomPushButton* getSaveButton(void) const { return saveButton; }
    CustomPushButton* getCancelButton(void) const { return cancelButton; }
    void setCreatingGroup(const bool creating) { creatingGroup = creating; }
    CustomizedLineEdit* getModifyEdit(void) const { return modifyEdit; }
    CustomizedLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }
    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }

    void disableButtons();

private:
    QHBoxLayout* creationLayout;
    CustomScrollArea* scrollArea;
    QLabel* groupNameLabel;
    CustomizedLineEdit* modifyEdit;
    CustomizedLineEdit* groupNameEdit;
    QSharedPointer<Paths> paths;
    QVBoxLayout* layout;
    GroupsPathsButtonGroup* buttonGroup;
    PathButtonGroup* pathButtonGroup;
    TopLeftMenu* actionButtons;
    QString lastCheckedButton;
    CustomPushButton* saveButton;
    CustomPushButton* cancelButton;
    /// to differenciate the behavior of the enter key
    bool creatingGroup;

private slots:
    void enableButtons(QAbstractButton* button);

};

#endif // GROUPSPATHSWIDGET_H
