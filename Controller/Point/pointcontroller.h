#ifndef POINTCONTROLLER_H
#define POINTCONTROLLER_H

class Points;
class MainController;

#include <QObject>
#include <QVariant>

class PointController : public QObject {
    Q_OBJECT
public:
    PointController(QObject *applicationWindow, MainController *parent);

    /// Getters
    Points* getPoints(void) const { return points; }

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
     * @brief indexOfPoint
     * @param pointName
     * @param groupName
     * @return the index of the given point as in the qml point list
     */
    int indexOfPoint(QString pointName, QString groupName);

    /**
     * @brief indexOfGroup
     * @param groupName
     * @return the index of the given group as in the qml point list
     */
    int indexOfGroup(QString groupName);

    /**
     * @brief checkPointName
     * @param name
     * @return if the given name of point is taken
     */
    bool checkPointName(const QString name);

    /**
     * @brief checkGroupName
     * @param name
     * @return if the given name of group is taken
     */
    bool checkGroupName(const QString name);

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
    void addPoint(QString name, QString groupName, double x, double y, QString oldName = "", QString oldGroup = "", bool displayed = true, bool saveXML = true);

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
    void enablePointSaveQml(QVariant enable);

    /**
     * @brief enableGroupSaveQml
     * @param enable
     * Signal to enable the save button while creating/editing a group
     */
    void enableGroupSaveQml(QVariant enable);

    /**
     * @brief addGroupQml
     * @param index
     * @param name
     * Tell the qml model that we added a new group
     */
    void addGroupQml(QVariant index, QVariant name);

    /**
     * @brief addPointQml
     * @param index
     * @param name
     * @param isVisible
     * @param groupName
     * @param x
     * @param y
     * Tell the point model that we added a new point
     */
    void addPointQml(QVariant index, QVariant name, QVariant isVisible, QVariant groupName, QVariant x, QVariant y);
    void editPointQml(QVariant oldName, QVariant oldGroup, QVariant index, QVariant name, QVariant isVisible, QVariant groupName, QVariant x, QVariant y);

    /**
     * @brief renameGroupQml
     * @param newName
     * @param oldName
     * Tell the qml model that we renamed the group oldName into newName
     */
    void renameGroupQml(QVariant newName, QVariant oldName);

private:
    Points* points;
    QString currentPointsFile;
};

#endif // POINTCONTROLLER_H
