#ifndef GROUPSPATHSWIDGET_H
#define GROUPSPATHSWIDGET_H

#include <QWidget>
#include "Model/paths.h"

class CustomizedLineEdit;
class GroupsPathsButtonGroup;
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

    GroupsPathsButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    CustomPushButton* getSaveButton(void) const { return saveButton; }
    CustomPushButton* getCancelButton(void) const { return cancelButton; }
    void setCreatingGroup(const bool creating) { creatingGroup = creating; }
    CustomizedLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }
    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }

    void setLastCheckedButton(const QString textButton) { lastCheckedButton = textButton; }

    void disableButtons();
    QString formatName(const QString name) const;
    void updateGroupsPaths(void);
    void uncheck(void);
    void enableActionButtons(void);
    void hideCreationWidgets(void);
    /// sets the widget in the state where u can either click a group or create a new one but nothing else
    /// same state as when u show the widget
    void resetWidget(void);

protected:
    void keyPressEvent(QKeyEvent* event);
    void hideEvent(QHideEvent *event);

signals:
    void newPathGroup(QString);
    void messageCreationGroup(QString, QString);
    void codeEditGroup(int);
    void modifiedGroup(QString);

public slots:
    int checkGroupName(QString name);
    int checkEditGroupName(QString name);
    void cancelCreationGroup();

private slots:
    void enableButtons(QAbstractButton* button);
    void newGroupPaths();


private:
    MainWindow* parent;

    QHBoxLayout* creationLayout;
    CustomScrollArea* scrollArea;
    QLabel* groupNameLabel;

    CustomizedLineEdit* groupNameEdit;
    QSharedPointer<Paths> paths;
    QVBoxLayout* layout;
    GroupsPathsButtonGroup* buttonGroup;
    TopLeftMenu* actionButtons;
    QString lastCheckedButton;
    CustomPushButton* saveButton;
    CustomPushButton* cancelButton;
    /// to differenciate the behavior of the enter key
    bool creatingGroup;


};

#endif // GROUPSPATHSWIDGET_H
