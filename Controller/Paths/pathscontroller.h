#ifndef PATHSCONTROLLER_H
#define PATHSCONTROLLER_H

#include <QObject>
#include "Model/Paths/paths.h"
#include "View/Paths/pathpainter.h"
#include "View/Paths/displaypathgroup.h"
#include "View/Paths/displayselectedpath.h"
#include "View/Paths/groupspathswidget.h"
#include "View/Paths/pathcreationwidget.h"
#include "View/Paths/groupspathsbuttongroup.h"
#include "View/Other/custompushbutton.h"
#include "Controller/mainwindow.h"

class PathPoint;

class PathsController: public QObject
{
    Q_OBJECT

public:

    PathsController(MainWindow *mainWindow);

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

    void updateDisplayedPath();

    void enableSaveEditButton(const bool enable);

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
    void exhibitDisplayedPath(void);
    void setPathsGroup(const QString groupName);
    void updateGroupsPaths(void) { groupsPathsWidget->getButtonGroup()->updateButtons(paths); }

private slots:
    void displayPathSlot(const QString group, const QString path, const bool display);
    void editGroupPaths();
    /// the path received corresponds to a button in the group
    void checkEyeButtonSlot(const QString path);
    void updateConnectionsRequestSlot();
    void setMessageModifGroupPaths(int code);

    /**
     * @brief checkPathGroupName
     * @param name
     * @return
     * to make sure that the name chosen to create a group is valid
     */
    int checkPathGroupName(QString name);
    void saveGroupPaths(QString name);

    /**
     * @brief checkEditGroupName
     * @param name
     * @return
     * to make sure that the new name chosen for an existing group is valid
     */
    void checkEditGroupName(QString name);

    /**
     * @brief checkPathName
     * @param name
     * checks that the name of a path point is valid (not already taken or empty
     */
    void checkPathName(const QString name);
    void setMessageNoRobotPath(const int code);

signals:
    void setMessageTop(QString,QString);
    void setTemporaryMessageTop(QString type, QString message, int ms);
    void enableReturnAndCloseButtons();
    void resetPath();
    void setCurrentPath(QVector<QSharedPointer<PathPoint>>, QString);
    void updatePathPainter(bool);

private:

    QSharedPointer<Paths> paths;

    DisplaySelectedPath* displaySelectedPath;
    GroupsPathsWidget* groupsPathsWidget;
    DisplayPathGroup* pathGroup;
    PathCreationWidget* pathCreationWidget;

    PathPainter* pathPainter;
};

#endif /// PATHSCONTROLLER_H
