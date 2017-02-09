#ifndef PATHSCONTROLLER_H
#define PATHSCONTROLLER_H

#include <QObject>
#include "Model/paths.h"
#include "View/pathpainter.h"
#include "View/displaypathgroup.h"
#include "View/displayselectedpath.h"
#include "View/groupspathswidget.h"
#include "View/pathcreationwidget.h"
#include "View/groupspathsbuttongroup.h"
#include "View/custompushbutton.h"
#include "Controller/mainwindow.h"

class PathPoint;

class PathsController: public QObject
{
    Q_OBJECT

public:

    PathsController(MainWindow *mainWindow, const QSharedPointer<Points> points);

    QSharedPointer<Paths> getPaths(void) const { return paths; }

    DisplaySelectedPath* getDisplaySelectedPath(void) const { return displaySelectedPath; }
    GroupsPathsWidget* getGroupsPathsWidget(void) const { return groupsPathsWidget; }
    DisplayPathGroup* getPathGroupDisplayed(void) const { return pathGroup; }
    PathCreationWidget* getPathCreationWidget(void) const { return pathCreationWidget; }
    PathPainter* getPathPainter(void) const { return pathPainter; }

    void initializePaths(void);
    void serializePaths(const QString fileName);
    void deserializePaths(const QString fileName);

    bool deletePath(const QString groupName, const QString pathName) { return paths->deletePath(groupName, pathName); }
    bool createPath(const QString groupName, const QString pathName) { return paths->createPath(groupName, pathName); }

    bool createGroup(const QString name) { return paths->createGroup(name); }
    int deleteGroup(const QString name) { return paths->deleteGroup(name); }

    void addPathPoint(const QString groupName, const QString pathName, const QSharedPointer<PathPoint> &pathPoint) { paths->addPathPoint(groupName, pathName, pathPoint); }

    QString getVisiblePath(void) const { return pathPainter->getVisiblePath(); }
    void setVisiblePath(const QString pathName) { pathPainter->setVisiblePath(pathName); }

    void updateDisplayedPath(const QString groupName, const QString pathName, const QVector<QSharedPointer<PathPoint>>& path, const QString visiblePath) { displaySelectedPath->updatePath(groupName, pathName, path, visiblePath); }

    void enableSaveEditButton(const bool enable);

    void updateGroupsPaths(void) { groupsPathsWidget->updateGroupsPaths(); }

    void showGroupsPathsWidget(void) const { groupsPathsWidget->show(); }
    void showPathsGroup(void) const { pathGroup->show(); }

    void hideDisplayedPathWidget(void) const { displaySelectedPath->hide(); }
    void hideGroupsPathsWidget(void) const { groupsPathsWidget->hide(); }
    void hidePathCreationWidget(void) const { pathCreationWidget->hide(); }
    void hidePathGroupWidget(void) const { pathGroup->hide(); }

    void showPathCreationWidget(void) const { pathCreationWidget->show(); }

    void setLastPathChecked(const QString path) { pathGroup->setLastCheckedButton(path); }

    QString getGroupPathsChecked(void) const { return groupsPathsWidget->getButtonGroup()->getButtonGroup()->checkedButton()->text(); }

    void resetGroupsPathsWidget(void) const { groupsPathsWidget->resetWidget(); }

    int checkGroupPathName(const QString name) { return groupsPathsWidget->checkGroupName(name); }

    void hideGroupCreationWidgets(void) const;

    void enableGroupsPathsWidgetPlusButtonOnly(void) const;

    Paths::Path getPath(const QString group, const QString path, bool& found) { return paths->getPath(group, path, found); }

    void enablePathCreationSaveButton(const bool enable) { pathCreationWidget->getSaveButton()->setEnabled(enable); }

    QVector<QSharedPointer<PathPoint>> getCurrentPathFromPathPainter(void) { return pathPainter->getCurrentPath(); }

    void clearPaths(void) { paths->clear(); }

    QPair<QString, QString> findPath(const QVector<PathPoint>& path) { return paths->findPath(path); }

    void editPath(const QString group, const QString path);
    QString editPath(void);

    /// returns true if the path existed before
    bool deletePath();

    void displayGroupPaths();

    void prepareGroupPathsCreation(void);

    bool modifyGroupPathsWithEnter(QString name);

    void updatePaths(const Point& old_point, const Point& new_point);

    void displayPath(const QString groupName);

public slots:
    void doubleClickOnPathsGroup(const QString checkButton);
    void doubleClickOnPath(const QString pathName, const QString groupName);
    void updateDisplayedPath(void);
    void setPathsGroup(const QString groupName);

private slots:
    void displayPathSlot(const QString group, const QString path, const bool display);
    void editGroupPaths();
    /// the path received corresponds to a button in the group
    void checkEyeButtonSlot(const QString path);

private:

    QSharedPointer<Paths> paths;

    DisplaySelectedPath* displaySelectedPath;
    GroupsPathsWidget* groupsPathsWidget;
    DisplayPathGroup* pathGroup;
    PathCreationWidget* pathCreationWidget;

    PathPainter* pathPainter;
};

#endif /// PATHSCONTROLLER_H
