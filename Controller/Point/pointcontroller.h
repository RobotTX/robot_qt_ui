#ifndef POINTCONTROLLER_H
#define POINTCONTROLLER_H

class Points;
class MainController;

#include <QObject>
#include <QVariant>
#include <QPointer>

class PointController : public QObject {

    Q_OBJECT

public:
    PointController(QObject *applicationWindow, MainController *parent);

    /// Getter
    QPointer<Points> getPoints(void) const { return points; }

    /**
     * @brief checkErrorPoint
     * @param mapImage
     * @param name
     * @param x
     * @param y
     * Check if there is any error in the point name or position
     */
    void checkErrorPoint(const QImage &mapImage, const QString name, const QString oldName, const double x, const double y);

    /**
     * @brief checkPointName
     * @param name
     * @return if the given name of point is taken
     */
    bool checkPointName(const QString name);

    QVector<double> getHome(void);

    /**
     * @brief clearPoints
     * Delete all the points on the c++ and qml side
     */
    void clearPoints(void);

public slots:
    /**
     * @brief addPoint
     * @param name
     * @param groupName
     * @param x
     * @param y
     * @param displayed
     * Add a point to the model
     */
    void addPoint(const QString name, const QString groupName, const double x, const double y, const QString oldName = "", const QString oldGroup = "", const bool displayed = true, const bool home = false, const int orientation = 0, const bool saveXML = true);

    /**
     * @brief addGroup
     * @param groupName
     * Add a group to the model
     */
    void addGroup(QString groupName, bool saveXML = true);

private:
    /**
     * @brief loadPoints
     * @param fileName
     * Load the points form the XML file
     */
    void loadPoints(const QString fileName);

private slots:
    /**
     * @brief checkGroup
     * @param name
     * Check if the given name is already taken by a group and send a signal to qml
     */
    void checkGroup(QString name);

    /**
     * @brief deletePoint
     * @param name
     * @param groupName
     * Delete a point from the model
     */
    void deletePoint(QString groupName, QString name);

    /**
     * @brief deleteGroup
     * @param name
     * Delete a group and its points from the model
     */
    void deleteGroup(QString name);

    /**
     * @brief hideShow
     * @param name
     * @param groupName
     * @param show
     * Hide or show the given point
     */
    void hideShow(QString groupName, QString name);

    /**
     * @brief renameGroup
     * @param newName
     * @param oldName
     * Rename a group from oldName to newName
     */
    void renameGroup(QString newName, QString oldName);

    /**
     * @brief moveTo
     * @param name
     * @param oldName
     * @param newGroup
     * Move a point from oldGroup to newGroup
     */
    void moveTo(QString name, QString oldGroup, QString newGroup);

signals:
    /**
     * @brief enablePointSaveQml
     * @param enable
     * Signal to enable the save button while creating/editing a point
     */
    void enablePointSaveQml(QVariant posError, QVariant nameError);

    /**
     * @brief enableGroupSaveQml
     * @param enable
     * Signal to enable the save button while creating/editing a group
     */
    void enableGroupSaveQml(QVariant enable);

    /**
     * @brief addGroupQml
     * @param name
     * Tell the qml model that we added a new group
     */
    void addGroupQml(QVariant name);

    /**
     * @brief addPointQml
     * @param name
     * @param isVisible
     * @param groupName
     * @param x
     * @param y
     * Tell the qml model that we added a new point
     */
    void addPointQml(QVariant name, QVariant isVisible, QVariant groupName, QVariant x, QVariant y, QVariant home, QVariant orientation);

    /**
     * @brief editPointQml
     * @param oldName
     * @param oldGroup
     * @param name
     * @param isVisible
     * @param groupName
     * @param x
     * @param y
     * Tell the qml model that we edited a point
     */
    void editPointQml(QVariant oldName, QVariant oldGroup, QVariant name, QVariant isVisible, QVariant groupName, QVariant x, QVariant y, QVariant home, QVariant orientation);

    /**
     * @brief renameGroupQml
     * @param newName
     * @param oldName
     * Tell the qml model that we renamed the group oldName into newName
     */
    void renameGroupQml(QVariant newName, QVariant oldName);

    /**
     * @brief deleteGroupQml
     * @param groupName
     * Tells the qml model to delete the group <groupName>
     */
    void deleteAllGroupsQml();

private:
    QPointer<Points> points;
    QString currentPointsFile;
    QString robotPointsFile;
};

#endif /// POINTCONTROLLER_H
