#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

class MainController;

#include <QObject>
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

private:
    QPointer<Paths> paths;
    QString currentPathsFile;
};

#endif /// PATHCONTROLLER_H
