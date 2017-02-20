#ifndef SCANMAPLISTITEMWIDGET_H
#define SCANMAPLISTITEMWIDGET_H

class QGraphicsScene;
class QPushButton;
class QLabel;
class QLineEdit;
class ScanMapGraphicsItem;
class QGraphicsItem;
class Robots;

#include <QWidget>
#include <QSlider>

class ScanMapListItemWidget : public QWidget {

    Q_OBJECT

public:
    ScanMapListItemWidget(const int _id, const QString name, QSharedPointer<Robots> robots, QGraphicsScene *_scene);

    /// Setter
    void setId(const int _id){ id = _id; }

    /// Getters
    QString getRobotName(void) const { return robotName; }
    ScanMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }

    /**
     * @brief robotConnected
     * @param connected
     * Display an icon when the robot disconnect/reconnect
     */
    void robotConnected(const bool connected);

    /**
     * @brief robotScanning
     * @param scanning
     * Change the play/pause button icon and label when a robot start/stop scanning
     */
    void robotScanning(const bool scanning);

    /**
     * @brief updateMap
     * @param map
     * Update the pixmap with the new given map
     */
    void updateMap(QImage map);

    /**
     * @brief updateRobotPos
     * @param x
     * @param y
     * @param ori
     * Update the position of the robot
     */
    void updateRobotPos(double x, double y, double ori);

protected:
    /**
     * @brief cropImage
     * @param image
     * @return
     * When we receive an image, we crop it, keep only the walls and floor, and change the color of the walls
     */
    QImage cropImage(const QImage &image);

    /**
     * @brief mouseDoubleClickEvent
     * To be able to double click the widget which will center the view on the robot view
     */
    void mouseDoubleClickEvent(QMouseEvent *);

private slots:
    /**
     * @brief sliderSlot
     * @param value
     * Called when the slider has been moved
     */
    void sliderSlot(int value);

    /**
     * @brief closeBtnSlot
     * Called when we close the widget
     */
    void closeBtnSlot();

    /**
     * @brief rotLineEditSlot
     * @param text
     * Called when the QEditLine for the rotation of the map has been changed
     */
    void rotLineEditSlot(QString text);

    /**
     * @brief scanningBtnSlot
     * Called when the user click on the button to play/pause a scan
     */
    void scanningBtnSlot();

    /**
     * @brief robotGoToSlot
     * @param x
     * @param y
     * Called when a user middle mouse clicked on the map to tell the robot where to go
     */
    void robotGoToSlot(double x, double y);

signals:
    /**
     * @brief deleteMap
     * Signal that we want to delete this map
     */
    void deleteMap(int, QString);

    /**
     * @brief playScan
     * Signal that we play/pause the scan of the given robot
     */
    void playScan(bool, QString);

    /**
     * @brief robotGoTo
     * Signal that we want the given robot to go to this point
     */
    void robotGoTo(QString, double, double);

    /**
     * @brief centerOn
     * Signal that we want to center the image on the given QGraphicsItem
     */
    void centerOn(QGraphicsItem*);

private:
    int id;
    QString robotName;
    QGraphicsScene* scene;
    ScanMapGraphicsItem* pixmapItem;
    QPushButton* closeBtn;
    QSlider* slider;
    QLineEdit* rotLineEdit;
    QLabel* fileNameLabel;
    QPushButton* discoIcon;
    QPushButton* warningIcon;
    QPushButton* scanningBtn;
    QLabel* scanningLabel;

    /// the amount we crop in both dimensions
    int top;
    int left;
 };


#endif /// SCANMAPLISTITEMWIDGET_H
