#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

class QQmlApplicationEngine;
class MainMenuController;
class MapController;
class PointController;

#include <QObject>
#include <QList>

class MainController : public QObject {

    Q_OBJECT

public:

    MainController(QQmlApplicationEngine* _engine, QObject* parent = Q_NULLPTR);

private slots:
    /**
     * @brief checkPoint
     * @param name
     * @param x
     * @param y
     * To check if the point <name> located at (x, y) is within the known area of the map
     */
    void checkPoint(QString name, double x, double y);
    /**
     * @brief saveMapConfig
     * @param fileName the map file name (.pgm)
     * @param zoom
     * @param centerX
     * @param centerY
     * To export the current map in files (one for the image, one for the configuration, one for the points, one for the paths)
     */
    void saveMapConfig(QString fileName, double zoom, double centerX, double centerY) const;
    /**
     * @brief loadMapConfig
     * @param fileName configuration file
     * To import a map inside the application (along with its points, configuration and paths)
     */
    void loadMapConfig(QString fileName) const;

private:
    MainMenuController* mainMenuController;
    MapController* mapController;
    PointController* pointController;
};

#endif /// MAINCONTROLLER_H
