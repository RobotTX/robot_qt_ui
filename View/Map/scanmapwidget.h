#ifndef SCANMAPWIDGET_H
#define SCANMAPWIDGET_H

class QHBoxLayout;
class CustomQGraphicsView;
class MergeMapListWidget;

#include <QDialog>
#include <QGraphicsScene>
#include "Model/Robots/robots.h"
#include "Model/Other/position.h"

#define LIST_WIDGET_HEIGHT 140

class ScanMapWidget : public QDialog {

    Q_OBJECT

public:

    ScanMapWidget(QSharedPointer<Robots> _robots, QWidget* parent = Q_NULLPTR);

    /**
     * @brief getAllScanningRobots
     * @return the list of all scanning robots' name
     */
    QStringList getAllScanningRobots();

protected:
    /**
     * @brief initializeMenu.Initialize the menu
     */
    void initializeMenu();

    /**
     * @brief initializeMap
     * Initiliaze the scene and graphics view
     */
    void initializeMap();

    /**
     * @brief addMapWidget
     * @param name
     * Create a new ScanMapListItemWidget to add to the list
     */
    void addMapWidget(QString name);

    /**
     * @brief closeEvent
     * @param event
     * Called when we close the widget
     */
    void closeEvent(QCloseEvent *event);

    /**
     * @brief refreshIds
     * Refresh the ids of the ScanMapListItemWidget according to their position in the list
     */
    void refreshIds();

    /**
     * @brief sceneToImage
     * @return
     * Combine all the items on the scene into a single QImage
     */
    QImage sceneToImage();

    /**
     * @brief croppedImageToMapImage
     * @param croppedImage
     * @return
     * Transform the image from sceneToImage to a new image with the right size
     */
    QImage croppedImageToMapImage(const QImage &croppedImage);

    /**
     * @brief checkImageSize
     * @param sizeCropped
     * @return
     * If the croped image is bigger than the image we want, ask the user if he still wants to continue
     * even if he might lose some informations
     */
    bool checkImageSize(QSize sizeCropped);

    /**
     * @brief keyPressEvent
     * @param event
     * Called when any key is pressed
     */
    void keyPressEvent(QKeyEvent *event);

private slots:
    /**
     * @brief cancelSlot
     * Clicked on the cancel button
     */
    void cancelSlot();

    /**
     * @brief saveSlot
     * Clicked on the save button
     */
    void saveSlot();

    /**
     * @brief addImageRobotSlot
     * Open the menu listing the available robots
     */
    void addImageRobotSlot();

    /**
     * @brief robotMenuSlot
     * @param action
     * Called when a robot has been chosen to scan
     */
    void robotMenuSlot(QAction* action);

    /**
     * @brief startedScanningSlot
     * @param robotName
     * @param scanning
     * Called when a robot successfully started scanning
     */
    void startedScanningSlot(QString robotName, bool scanning);

    /**
     * @brief robotDisconnectedSlot
     * @param robotName
     * Caleld when a robot disconnected
     */
    void robotDisconnectedSlot(QString robotName);

    /**
     * @brief deleteMapSlot
     * @param id
     * @param robotName
     * Called when we want to stop the scan and delete the map of a robot
     */
    void deleteMapSlot(int id, QString robotName);

    /**
     * @brief robotReconnectedSlot
     * @param robotName
     * Called when a robot reconnected
     */
    void robotReconnectedSlot(QString robotName);

    /**
     * @brief playScanSlot
     * @param scan
     * @param robotName
     * Called when a user click on the button to play/pause a scan
     */
    void playScanSlot(bool scan, QString robotName);

    /**
     * @brief robotScanningSlot
     * @param scan
     * @param robotName
     * @param success
     * Called when a robot has changed its status and started/stopped scanning
     */
    void robotScanningSlot(bool scan, QString robotName, bool success);

    /**
     * @brief receivedScanMapSlot
     * @param robotName
     * @param map
     * @param _resolution
     * Called when we received a new map from a robot
     */
    void receivedScanMapSlot(QString robotName, QImage map, double _resolution);

    /**
     * @brief robotGoToSlot
     * @param robotName
     * @param x
     * @param y
     * Called when we want to tell a robot where to go
     */
    void robotGoToSlot(QString robotName, double x, double y);

    /**
     * @brief scanRobotPosSlot
     * @param robotName
     * @param x
     * @param y
     * @param ori
     * Called when we receive a new robot position
     */
    void scanRobotPosSlot(QString robotName, double x, double y, double ori);

    /**
     * @brief centerOnSlot
     * @param pixmap
     * Called when the user double clicked a widget in the list
     */
    void centerOnSlot(QGraphicsItem* pixmap);

    /**
     * @brief teleopCmdSlot
     * Called when a user pressed one of the button in the teleop widget or pressed a key
     */
    void teleopCmdSlot(int);

    /**
     * @brief dirKeyEventSlot
     * @param key
     * Called when the user pressed a key
     */
    void dirKeyEventSlot(int key);

signals:
    /**
     * @brief startScanning
     * To Start from the beggining a scan
     */
    void startScanning(QString);

    /**
     * @brief stopScanning
     * To completely stop a scan
     */
    void stopScanning(QStringList);

    /**
     * @brief playScan
     * To play/pause a scan that is already launched
     */
    void playScan(bool, QString);
    void robotGoTo(QString, double, double);
    void saveScanMap(double, Position, QImage, QString);
    void teleopCmd(QString, int);

private:
    QSharedPointer<Robots> robots;
    QHBoxLayout* layout;
    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    MergeMapListWidget* listWidget;
    QSize mapSize;
    double resolution;
};

#endif /// SCANMAPWIDGET_H
