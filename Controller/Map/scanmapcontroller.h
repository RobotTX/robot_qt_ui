#ifndef SCANMAPCONTROLLER_H
#define SCANMAPCONTROLLER_H

class MainController;
class QQmlApplicationEngine;

#include <QObject>
#include <QImage>
#include <QVariant>
#include "View/Robot/scanmappainteditem.h"

class ScanMapController : public QObject  {

    Q_OBJECT

public:

    ScanMapController(MainController* parent, QQmlApplicationEngine* engine, QObject *applicationWindow);

    /**
     * @brief receivedScanMap
     * @param ip
     * @param map
     * @param resolution
     * adds a new map to our list of maps or update it if it was already existing
     */
    void receivedScanMap(QString ip, QImage map, QString resolution);
    /**
     * @brief updateRobotPos
     * @param ip
     * @param x
     * @param y
     * @param orientation
     * updates the robot position whose ip is <ip> to (x, y) with orientation <orientation>
     */
    void updateRobotPos(QString ip, double x, double y, double orientation);

    QMap<QString, ScanMapPaintedItem*> getPaintedItems(void) const { return paintedItems; }

public slots:
    /**
     * @brief removeMap
     * @param ip
     * removes the map received by the robot at ip <ip>
     */
    void removeMap(QString ip);

private slots:
    /**
     * @brief saveScanSlot
     * @param file_name
     * draws the pixels the right colors (black, white, grey),
     * hides the robot and finally notify the qml side that this the map is ready to be saved
     * in the file <file_name>
     */
    void saveScanSlot(QString file_name);
    /**
     * @brief resetScanMaps
     * hides all the maps to reset the content of the window
     */
    void resetScanMaps();
    /**
     * @brief sendGoalSlot
     * @param ip
     * @param x
     * @param y
     * sends a goal (x, y) to the robot at ip <ip>
     */
    void sendGoalSlot(QString ip, double x, double y);
    /**
     * @brief sendCoordinatesRobotView
     * @param ip
     * Send the coordinates of the robot at ip <ip>
     */
    void sendCoordinatesRobotAndScanMapItem(QString ip);

signals:
    /**
     * @brief receivedScanMap
     * notifies the qml side that the map has been received in order
     * to change the icon in the menu on the left to notify the user
     */
    void receivedScanMap(QVariant);
    /**
     * @brief readyToBeGrabbed
     * notifies the qml side that the map is ready to be saved into the file
     * given as a parameter
     */
    void readyToBeGrabbed(QVariant);
    /**
     * @brief sendGoal
     * sends the robot with ip <ip> to position (x, y)
     */
    void sendGoal(QString ip, double x, double y);
    /**
     * @brief invalidGoal
     * notifies the qml side that the goal chosen is invalid (not a known area of the map)
     */
    void invalidGoal();
    /**
     * @brief updateSize
     * Sends the size of the map to the merge and scan widgets so that they can save maps
     * of the same size
     */
    void updateSize(QVariant, QVariant);
    /**
     * @brief setMessageTop
     * @param status
     * @param msg
     * To display an informative message at the top of the application
     */
    void setMessageTop(int status, QString msg);
    /**
     * @brief clearPointsAndPaths
     * After the map is saved it is going to replace the current map inside the main window
     * For this reason the points and paths must be cleared
     */
    void clearPointsAndPaths();
    /**
     * @brief discardMap
     * After the map is saved maps keep arriving so we send this signal to notify the main controller
     * to discard them
     */
    void discardMap(bool);
    /**
     * @brief coordinatesRobotView
     * Coordinates of the robot view and scan map item that we were requested to send in order to
     * center the scan map on this particular robot
     */
    void coordinatesRobotAndScanMapItem(QVariant, QVariant, QVariant, QVariant);

private:
    QMap<QString, ScanMapPaintedItem*> paintedItems;
    /// to store the color of the maps
    QMap<QString, int> colors;
    QQmlApplicationEngine* engine;
    QObject* applicationWindow;
};

#endif /// SCANMAPCONTROLLER_H
