#ifndef MERGEMAPWIDGET_H
#define MERGEMAPWIDGET_H

class QHBoxLayout;
class CustomQGraphicsView;
class QButtonGroup;
class MergeMapListWidget;
class MergeMapListItemWidget;


#include <QWidget>
#include <QGraphicsScene>
#include "Model/robots.h"
#include "Model/position.h"

#define MERGE_WIDGET_HEIGHT 120

class MergeMapWidget : public QWidget {
    Q_OBJECT
public:
    MergeMapWidget(QSharedPointer<Robots> _robots, QWidget* parent = Q_NULLPTR);

protected:
    void initializeMenu();
    void initializeMap();

    /**
     * @brief refreshIds
     * Refresh the ids of the maps when a map is deleted
     */
    void refreshIds();

    /**
     * @brief sceneToImage
     * @return a QImage from the scene
     */
    QImage sceneToImage();

    /**
     * @brief croppedImageToMapImage
     * @param croppedImage
     * @return
     * Transform a cropped image to an image with the size it is supposed to have
     */
    QImage croppedImageToMapImage(QImage croppedImage);

    /**
     * @brief checkImageSize
     * @param sizeCropped
     * @return
     * Check and alert the user if the cropped image is bigger than the one we are going to output
     */
    bool checkImageSize(QSize sizeCropped);

    /**
     * @brief getResolution
     * Try to get the resolution from the different map we imported
     * If it can't find it there, try to get it from the curretnly used map in the mainWindow
     * else alert the user that we couldn't find a resolution
     */
    void getResolution();

    /**
     * @brief addMap
     * @param fileName
     * @param fromRobot
     * @param image
     * @param _resolution
     * @param _originX
     * @param _originY
     * Create a MergeMapListItemWidget and add it to the list
     */
    void addMap(QString fileName, bool fromRobot, QImage image = QImage(), double _resolution = -1, double _originX = -1, double _originY = -1);

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
     * @brief resetSlot
     * Clicked on the reset button
     */
    void resetSlot();

    /**
     * @brief addImageFileSlot
     * Clicked on the button to add a map from a file
     */
    void addImageFileSlot();

    /**
     * @brief addImageRobotSlot
     * Clicked on the button to add a map from a robot
     */
    void addImageRobotSlot();

    /**
     * @brief deleteMapSlot
     * @param itemId
     * Clicked on the close button of one of the widget in the list of map
     */
    void deleteMapSlot(int itemId);

    /**
     * @brief dirKeyEventSlot
     * @param key
     * Pressed a directional key which will move the selected map
     */
    void dirKeyEventSlot(int key);

    /**
     * @brief selectPixmap
     * @param id
     * When a user clicked on a pixmap on the scene, we select it on the list
     */
    void selectPixmap(int id);

    /**
     * @brief robotMenuSlot
     * When the user click on the button to add a map from a robot,
     * we open a menu to select a robot which will then call this slot when a robot is selected
     * and send a signal to the mainwindow to try to get the map
     */
    void robotMenuSlot(QAction*);

    /**
     * @brief receivedMapToMergeSlot
     * @param robotName
     * @param image
     * @param _resolution
     * @param _originX
     * @param _originY
     * We received a map from a robot to merge
     */
    void receivedMapToMergeSlot(QString robotName, QImage image, double _resolution, double _originX, double _originY);

signals:
    /**
     * @brief saveMergeMap
     * Tell the mainWindow that we finished saving
     */
    void saveMergeMap(double, Position, QImage, QString);

    /**
     * @brief getMapForMerging
     * Tell the mainWindow that we want the map from a robot
     */
    void getMapForMerging(QString);

private:
    MergeMapListWidget* listWidget;
    QHBoxLayout* layout;
    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    QButtonGroup* sizeGroup;
    QSize originalSize;
    double resolution;
    QPoint croppedOriginInPixel;
    QPoint originInPixel;
    QSharedPointer<Robots> robots;
};

#endif // MERGEMAPWIDGET_H
