#ifndef POINTS_H
#define POINTS_H

class QDataStream;
class MapView;
class MainWindow;

#include <QVector>
#include <QMap>
#include <QString>
#include <QSharedPointer>
#include <iostream>
#include "point.h"
#include <QObject>
#include "Model/Other/graphicitemstate.h"
#include "View/Points/pointview.h"

#define NO_GROUP_NAME "No Group"
#define TMP_GROUP_NAME "TmpPoint"
#define PATH_GROUP_NAME "PathPoints"

/**
 * @brief The Points class
 * This class provides a model for a list of points organized in groups
 * A Points objet is identified by a vector of pointers on Group objets
 */

class Points : public QObject {

    Q_OBJECT

public:
    Points(QObject *parent, MainWindow *_mainWindow);
    typedef QMap<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> Groups;

    /// a helper class to overload the << operator
    void display(std::ostream& stream) const;

public:

    QSharedPointer<Groups> getGroups(void) const { return groups; }
    /**
     * @brief getDefaultGroup
     * @return the default group
     */
    QSharedPointer<QVector<QSharedPointer<PointView>>> getDefaultGroup(void) const { return groups->value(NO_GROUP_NAME); }

    /**
     * @brief countGroups
     * @return the number of groups
     */
    int countGroups(void) const { return groups->count(); }

    /**
     * @brief count
     * @return the number of points
     */
    int count(void) const;

    /**
     * @brief addGroup
     * @param name
     * Add an empty group groupName
     */
    void addGroup(const QString groupName) { groups->insert(groupName, QSharedPointer<QVector<QSharedPointer<PointView>>>(new QVector<QSharedPointer<PointView>>)); }

    /**
     * @brief addGroup
     * @param groupName
     * @param points
     * Add the given groupName as a group containing the given points
     */
    void addGroup(const QString groupName, QSharedPointer<QVector<QSharedPointer<PointView>>> points);

    /**
     * @brief removeGroup
     * @param groupName
     * Remove the group groupName
     */
    void removeGroup(const QString groupName);

    /**
     * @brief removePoint
     * @param pointName
     * Remove the point pointName
     */
    void removePoint(const QString pointName);

    /**
     * @brief findGroup
     * @param groupName
     * @return the group groupName
     */
    QSharedPointer<QVector<QSharedPointer<PointView>>> findGroup(const QString groupName) const;

    /**
     * @brief findPoint
     * @param pointName
     * @return the point pointName
     */
    QSharedPointer<Point> findPoint(const QString pointName) const;

    /**
     * @brief findPoint
     * @param groupName
     * @param indexPoint
     * @return the point in the group groupName and in the given index
     */
    QSharedPointer<Point> findPoint(const QString groupName, const int indexPoint) const;

    /**
     * @brief findPointView
     * @param pointName
     * @return the pointView of pointName
     */
    QSharedPointer<PointView> findPointView(const QString pointName) const;

    /**
     * @brief findPathPointView
     * @param x
     * @param y
     * @return the pointView of a PathPoint in the position (x, y)
     */
    QSharedPointer<PointView> findPathPointView(const double x, const double y) const;

    /**
     * @brief findPointIndexes
     * @param pointName
     * @return the name of the group and index of the given pointName
     */
    QPair<QString, int> findPointIndexes(const QString pointName) const;

    /**
     * @brief clear
     * Clear all the groups
     */
    void clear();

    /**
     * @brief addPoint
     * @param groupName
     * @param pointName
     * @param x
     * @param y
     * @param displayed
     * @param type
     * @param mapView
     * @param mainWindow
     * Add a point with the given informations to the group groupName
     */
    void addPoint(const QString groupName, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type);

    /**
     * @brief displayTmpPoint
     * @param display
     * Display all the groups
     */
    void displayTmpPoint(const bool display);

    /**
     * @brief setPointViewsState
     * @param state
     * Set the state of all the pointViews
     */
    void setPointViewsState(const GraphicItemState state);

    /**
     * @brief getTmpPointView
     * @return the tmpPoint pointview
     */
    QSharedPointer<PointView> getTmpPointView();

    /**
     * @brief isDisplayed
     * @param groupName
     * @return whether or not the points in the group are all displayed
     */
    bool isDisplayed(const QString groupName) const;

    /**
     * @brief isAGroup
     * @param groupName
     * @return whether or not groupName is the name of a group
     */
    bool isAGroup(const QString groupName) const;

    /**
     * @brief isAPoint
     * @param pointName
     * @return whether or not pointName is the name of a point
     */
    bool isAPoint(const QString pointName) const;

    /**
     * @brief isAPoint
     * @param pointName
     * @param x
     * @param y
     * @return whether or not pointName in the pos (x, y) is the name of a point
     */
    bool isAPoint(const QString pointName, const double x, const double y) const;

    /**
     * @brief isAHome
     * @param pointName
     * @param x
     * @param y
     * @return whether or not pointName in the pos (x, y) is the name of a home of a robot
     */
    bool isAHome(const QString pointName, const double x, const double y) const;

    /**
     * @brief getHomeNameFromGroup
     * @param groupName
     * @return the names of homes in the given group
     */
    QVector<QString> getHomeNameFromGroup(const QString groupName) const;

    /**
     * @brief getGroupNameFromPointName
     * @param pointName
     * @return the name of the group of the given point name
     */
    QString getGroupNameFromPointName(const QString pointName) const;

    /**
     * @brief addTmpPoint
     * @param mapView
     * @param mainWindow
     * Add the tmpPoint
     */
    void addTmpPoint();

    /**
     * @brief addPoint
     * @param groupName
     * @param pointView
     * Add a given PointView to a group
     */
    void addPoint(const QString groupName, QSharedPointer<PointView> pointView);

    /**
     * @brief insertPoint
     * @param groupName
     * @param id
     * @param pointView
     * Insert the given PointView at the position id in groupName
     */
    void insertPoint(const QString groupName, const int id, QSharedPointer<PointView> pointView);

    /**
     * @brief insertPoint
     * @param groupName
     * @param id
     * @param pointName
     * @param x
     * @param y
     * @param displayed
     * @param type
     * Create and insert a PointView at the position id in groupName
     */
    void insertPoint(const QString groupName, const int id, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type);

    /**
     * @brief replacePoint
     * @param groupName
     * @param id
     * @param pointView
     * Replace the point at the position id in the group groupName by the given pointView
     */
    void replacePoint(const QString groupName, const int id, const QSharedPointer<PointView> &pointView);

    /**
     * @brief setPixmapAll
     * @param type
     * @param selectedRobot
     * Set the pixmap of all pointViews
     */
    void setPixmapAll(const PointView::PixmapType type);

    /**
     * @brief createPoint
     * @param pointName
     * @param x
     * @param y
     * @param displayed
     * @param type
     * @param mapView
     * @param mainWindow
     * @return a pointView created with the given informations
     */
    QSharedPointer<PointView> createPoint(const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type);

    /**
     * @brief updatePointViews
     * Update all the pointViews positions
     */
    void updatePointViews(void);

    /**
     * @brief findPointViewByPos
     * @param pos
     * @return QSharedPointer<PointView>
     * finds the pointview whose position is <pos>
     */
    QSharedPointer<PointView> findPointViewByPos(const Position& pos);

private:
    MainWindow* mainWindow;
    QSharedPointer<Groups> groups;
};

std::ostream& operator <<(std::ostream& stream, Points const& points);

#endif /// POINTS_H
