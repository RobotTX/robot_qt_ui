#ifndef POINTCONTROLLER_H
#define POINTCONTROLLER_H

class Points;
class MainController;

#include <QObject>
#include <QVariant>

class PointController : public QObject {
    Q_OBJECT
public:
    PointController(QObject *applicationWindow, QString mapFile, MainController *parent);

    /**
     * @brief checkErrorPoint
     * @param mapImage
     * @param name
     * @param x
     * @param y
     * Check if there is any error in the point name or position
     */
    void checkErrorPoint(const QImage &mapImage, const QString name, const double x, const double y);

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

private:
    Points* points;
};

#endif // POINTCONTROLLER_H
