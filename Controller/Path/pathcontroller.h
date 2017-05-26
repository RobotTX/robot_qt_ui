#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

class MainController;

#include <QObject>
#include <QVariant>
#include "Model/Path/paths.h"
#include "Model/Path/path.h"
#include "Model/Path/pathgroup.h"
#include "Model/Path/pathpoint.h"


class PathController : public QObject {

    Q_OBJECT

public:
    PathController(QObject *applicationWindow, MainController* parent);

    /// Getters
    QPointer<Paths> getPaths(void) const { return paths; }
    QVector<QPointer<PathPoint>> getPath(const QString groupName, const QString pathName) { return paths->getGroups().value(groupName)->getPaths().value(pathName)->getPathPointVector(); }

    /**
     * @brief clearPaths
     * Delete all the paths on the c++ and qml side
     */
    void clearPaths(void);

public slots:
    /**
     * @brief addGroup
     * @param groupName
     * @param saveXML
     * Add a group of paths
     */
    void addGroup(const QString groupName, const bool saveXML = true);

    /**
     * @brief addPath
     * @param groupName
     * @param name
     * @param saveXML
     * Add a path
     */
    void addPath(const QString groupName, const QString name, const bool saveXML = true);

    /**
     * @brief addPathPoint
     * @param groupName
     * @param pathName
     * @param name
     * @param x
     * @param y
     * @param waitTime
     * @param saveXML
     * Add a pathpoint
     */
    void addPathPoint(const QString groupName, const QString pathName, const QString name, const double x, const double y, const int waitTime, const bool saveXML = true);

    /**
     * @brief checkPosition
     * @param mapImage
     * @param index
     * @param x
     * @param y
     * Check if the given position of a path point is correct
     */
    void checkPosition(const QImage& mapImage, const int index, const double x, const double y);

private:
    /**
     * @brief loadPaths
     * @param fileName
     * Load the paths form the XML file
     */
    void loadPaths(const QString fileName);

private slots:
    /**
     * @brief deleteGroup
     * @param groupName
     * Delete a group of paths
     */
    void deleteGroup(const QString groupName);

    /**
     * @brief deletePath
     * @param groupName
     * @param name
     * Delete a path
     */
    void deletePath(const QString groupName, const QString name);

    /**
     * @brief deletePathPoint
     * @param groupName
     * @param pathName
     * @param name
     * Delete a path point
     */
    void deletePathPoint(const QString groupName, const QString pathName, const QString name);

    /**
     * @brief renameGroup
     * @param newName
     * @param oldName
     * Rename the group <oldName> in <newName>
     */
    void renameGroup(const QString newName, const QString oldName);

    /**
     * @brief checkGroup
     * @param name
     * Check if the given name is already taken by a group and send a signal to qml
     */
    void checkGroup(const QString name);

    /**
     * @brief moveTo
     * @param name
     * @param oldName
     * @param newGroup
     * Move a path from oldGroup to newGroup
     */
    void moveTo(const QString name, const QString oldGroup, const QString newGroup);

signals:
    /**
     * @brief addGroupQml
     * Tell the qml model to add a group
     */
    void addGroupQml(QVariant groupName);

    /**
     * @brief addPathQml
     * Tell the qml model to add a path
     */
    void addPathQml(QVariant name, QVariant groupName);

    /**
     * @brief addPathPointQml
     * Tell the qml model to add a path point
     */
    void addPathPointQml(QVariant name, QVariant pathName, QVariant groupName, QVariant x, QVariant y, QVariant waitTime);

    /**
     * @brief renameGroupQml
     * Tell the qml model to rename the group <oldName> in <newName>
     */
    void renameGroupQml(QVariant newName, QVariant oldName);

    /**
     * @brief setTmpValidPositionQml
     * Tell the qml model if the position of path point at <index> is <valid>
     */
    void setTmpValidPositionQml(QVariant index, QVariant valid);

    /**
     * @brief enableGroupSaveQml
     * @param enable
     * Signal to enable the save button while creating/editing a group
     */
    void enableGroupSaveQml(QVariant enable);

    /**
     * @brief deleteAllPathsQml
     * Tell the qml model to delete all the paths
     */
    void deleteAllPathsQml();

private:
    QPointer<Paths> paths;
    QString currentPathsFile;
};

#endif /// PATHCONTROLLER_H
