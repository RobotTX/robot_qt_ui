#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

class MainController;

#include <QObject>
#include <QVariant>
#include "Model/Path/paths.h"

class PathController : public QObject {
    Q_OBJECT
public:
    PathController(QObject *applicationWindow, MainController* parent);

    QPointer<Paths> getPaths(void) const { return paths; }

public slots:
    void addGroup(const QString groupName, bool saveXML = true);
    void addPath(const QString groupName, const QString name, bool saveXML = true);
    void addPathPoint(const QString groupName, const QString pathName, const QString name, const double x, const double y, const int waitTime, bool saveXML = true);

private:
    /**
     * @brief loadPaths
     * @param fileName
     * Load the paths form the XML file
     */
    void loadPaths(const QString fileName);

private slots:
    void deleteGroup(const QString groupName);
    void deletePath(const QString groupName, const QString name);
    void deletePathPoint(const QString groupName, const QString pathName, const QString name);
    void renameGroup(const QString newName, const QString oldName);

    /**
     * @brief checkGroup
     * @param name
     * Check if the given name is already taken by a group and send a signal to qml
     */
    void checkGroup(QString name);

    /**
     * @brief moveTo
     * @param name
     * @param oldName
     * @param newGroup
     * Move a path from oldGroup to newGroup
     */
    void moveTo(QString name, QString oldGroup, QString newGroup);

signals:
    void addGroupQml(QVariant);
    void addPathQml(QVariant, QVariant);
    void addPathPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant);
    void renameGroupQml(QVariant, QVariant);

    /**
     * @brief enableGroupSaveQml
     * @param enable
     * Signal to enable the save button while creating/editing a group
     */
    void enableGroupSaveQml(QVariant enable);

private:
    QPointer<Paths> paths;
    QString currentPathsFile;
};

#endif // PATHCONTROLLER_H
