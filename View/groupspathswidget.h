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

    void setLastCheckedButton(const QString textButton) { lastCheckedButton = textButton; }

    void disableButtons();
    QString formatName(const QString name) const;
    void updateGroupsPaths(void);


protected:
    void keyPressEvent(QKeyEvent* event);

signals:
    void newPathGroup(QString);
    void messageCreationGroup(QString, QString);

public slots:
    int checkGroupName(QString name);

private slots:
    void enableButtons(QAbstractButton* button);

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

};

#endif // GROUPSPATHSWIDGET_H
