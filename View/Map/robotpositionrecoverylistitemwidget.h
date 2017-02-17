#ifndef ROBOTPOSITIONRECOVERYLISTITEMWIDGET_H
#define ROBOTPOSITIONRECOVERYLISTITEMWIDGET_H

#include <QWidget>
#include <Model/Robots/robots.h>

class QGraphicsScene;
class QGraphicsItem;
class RecoverPositionMapGraphicsItem;
class QPushButton;
class QLabel;

class RobotPositionRecoveryListItemWidget : public QWidget {

        Q_OBJECT

public:
    RobotPositionRecoveryListItemWidget(const int _id, const QString name, QSharedPointer<Robots> robots, QGraphicsScene *_scene);

    QString getRobotName(void) const { return robotName; }

    RecoverPositionMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }

    /**
     * @brief robotConnected
     * @param connected
     * Display an icon when the robot disconnect/reconnect
     */
    void robotConnected(const bool connected);

    /**
     * @brief updateMap
     * @param map
     * Update the pixmap with the new given map
     */
    void updateMap(const QImage &map);

    /**
     * @brief updateRobotPos
     * @param x
     * @param y
     * @param ori
     * Update the position of the robot
     */
    void updateRobotPos(double x, double y, double ori);

    /**
     * @brief robotScanning
     * @param scanning
     * Change the play/pause button icon and label when a robot start/stop recovering its position
     */
    void robotRecovering(const bool recovering);

    /**
     * @brief cropImage
     * @param image
     * @return
     * When we receive an image, we crop it, keep only the walls and floor, and change the color of the walls
     */
    QImage cropImage(const QImage &image);

signals:
    void startRecovery(bool start, const QString robotName);
    /**
     * @brief centerOn
     * Signal to center the image on the given QGraphicsItem
     */
    void centerOn(QGraphicsItem*);
    /**
     * @brief deleteMap
     * Signal that we want to delete this map
     */
    void deleteMap(int, QString);
    /**
     * @brief robotGoTo
     * Signal that we want the given robot to go to this point
     */
    void robotGoTo(QString, double, double);

protected:
    /**
     * @brief mouseDoubleClickEvent
     * To be able to double click the widget which will center the view on the robot view
     */
    void mouseDoubleClickEvent(QMouseEvent *);

private slots:
    void startRecoverySlot();
    void closeBtnSlot();
    /**
     * @brief robotGoToSlot
     * @param x
     * @param y
     * Called when a user middle mouse clicked on the map to tell the robot where to go
     */
    void robotGoToSlot(double x, double y);

private:
    int id;
    QString robotName;
    QGraphicsScene* scene;
    RecoverPositionMapGraphicsItem* pixmapItem;

    QPushButton* disconnectedIcon;
    QPushButton* warningIcon;
    QPushButton* recoverPositionBtn;
    QPushButton* closeBtn;

    QLabel* recoverPositionLabel;
    QLabel* fileNameLabel;

    /// the amount we crop in both dimensions
    int top;
    int left;
};

#endif /// ROBOTPOSITIONRECOVERYLISTITEMWIDGET_H
