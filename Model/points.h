#ifndef POINTS_H
#define POINTS_H

class QDataStream;
class PointView;
class MapView;
class MainWindow;

#include <QVector>
#include <QMap>
#include <QString>
#include <QSharedPointer>
#include <iostream>
#include "point.h"
#include <QObject>
#include "Model/graphicitemstate.h"
#include "View/pointview.h"

#define NO_GROUP_NAME "No Group"
#define TMP_GROUP_NAME "TmpPoint"
#define PATH_GROUP_NAME "PathPoints"

/**
 * @brief The Points class
 * This class provides a model for a list of points organized in groups
 * A Points objet is identified by a vector of pointers on Group objets
 */

class Points : public QObject{
    Q_OBJECT

    typedef QMap<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> Groups;
public:
    Points(QObject* parent);

    /// a helper class to overload the << operator
    void display(std::ostream& stream) const;

public:
    QSharedPointer<Groups> getGroups(void) const { return groups; }
    int countGroups(void) const { return groups->count(); }
    int count(void) const;
    void addGroup(const QString groupName, QSharedPointer<QVector<QSharedPointer<PointView>>> points);
    void removeGroup(const QString groupName);
    void removePoint(const QString pointName);
    QSharedPointer<QVector<QSharedPointer<PointView>>> findGroup(const QString groupName) const;
    QSharedPointer<Point> findPoint(const QString pointName) const;
    QSharedPointer<Point> findPoint(const QString groupName, const int indexPoint) const;
    QSharedPointer<PointView> findPointView(const QString pointName) const;
    QSharedPointer<PointView> findPathPointView(const double x, const double y) const;
    QPair<QString, int> findPointIndexes(const QString pointName) const;
    QSharedPointer<QVector<QSharedPointer<PointView>>> getDefaultGroup(void) const { return groups->value(NO_GROUP_NAME); }
    void addGroup(const QString name) { groups->insert(name, QSharedPointer<QVector<QSharedPointer<PointView>>>(new QVector<QSharedPointer<PointView>>)); }
    void clear();
    void addPoint(const QString groupName, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type, MapView *mapView, MainWindow *mainWindow);
    void displayTmpPoint(const bool display);
    void setPointViewsState(const GraphicItemState state);
    QSharedPointer<PointView> getTmpPointView() const;
    bool isDisplayed(const QString key) const;
    bool isAGroup(const QString groupName) const;
    bool isAPoint(const QString pointName) const;
    bool isAPoint(const QString pointName, const double x, const double y) const;
    QVector<QString> getHomeNameFromGroup(const QString groupName) const;
    QString getGroupNameFromPointName(const QString pointName) const;
    void addTmpPoint(MapView *mapView, MainWindow *mainWindow);
    void addPoint(const QString groupName, QSharedPointer<PointView>pointView);
    void insertPoint(const QString groupName, const int id, QSharedPointer<PointView>pointView);
    void insertPoint(const QString groupName, const int id, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type, MapView *mapView, MainWindow *mainWindow);
    void setPixmapAll(const PointView::PixmapType type);
    void setPixmapAll(const QPixmap pixmap);
    QSharedPointer<PointView> createPoint(const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type,
                                                   MapView* mapView, MainWindow* mainWindow);
    void updatePointViews(void);

private:
    QSharedPointer<Groups> groups;
};


std::ostream& operator <<(std::ostream& stream, Points const& points);

/**
 * @brief operator <<
 * @param out
 * @param group
 * @return QDataStream&
 * Overloads the << and >> operators in order to be able to serialize a Group objet
 */
QDataStream& operator<<(QDataStream& out, const Points& points);
QDataStream& operator>>(QDataStream& in, Points& points);

#endif // POINTS_H
