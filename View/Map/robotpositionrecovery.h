#ifndef ROBOTPOSITIONRECOVERY_H
#define ROBOTPOSITIONRECOVERY_H

class Robots;
class QHBoxLayout;
class MergeMapListWidget;
class QGraphicsScene;
class CustomQGraphicsView;
class QGraphicsItem;

#include <QSharedPointer>
#include <QWidget>

#define LIST_WIDGET_HEIGHT 140

class RobotPositionRecovery : public QWidget {

    Q_OBJECT

public:
    RobotPositionRecovery(QSharedPointer<Robots> _robots, QWidget* parent = Q_NULLPTR);

    void initializeMenu(void);
    void initializeMap(void);

    QStringList getAllRecoveringRobots(void) const;

    void addMapWidget(QString name);

private slots:
    /**
     * @brief centerOnSlot
     * @param pixmap
     * Called when the user double clicked a widget in the list
     */
    void centerOnSlot(QGraphicsItem* pixmap);
    /**
     * @brief cancelSlot
     * Clicked on the cancel button
     */
    void cancelSlot();
    /**
     * @brief dirKeyEventSlot
     * @param key
     * Called when the user pressed a key
     */
    void dirKeyEventSlot(int key);

    /**
     * @brief teleopCmdSlot
     * Called when a user pressed one of the button in the teleop widget or pressed a key
     */
    void teleopCmdSlot(int id);

    void addImageRobotSlot(void);

    /**
     * @brief receivedScanMapSlot
     * @param robotName
     * @param map
     * @param _resolution
     * Called when we received a new map from a robot
     */
    void receivedMapSlot(QString robotName, QImage map, double _resolution);

    /**
     * @brief scanRobotPosSlot
     * @param robotName
     * @param x
     * @param y
     * @param ori
     * Called when we receive a new robot position
     */
    void updateRobotPosSlot(QString robotName, double x, double y, double ori);

    void robotGoToSlot(QString robotName, double x, double y);

    void startedRecoveringSlot(QString robotName, bool recovering);

    void robotMenuSlot(QAction* action);

    void robotDisconnectedSlot(QString robotName);
    void robotReconnectedSlot(QString robotName);

    void robotRecoveringSlot(bool recover, QString robotName, bool success);

    void playRecoverySlot(bool recover, QString robotName);

    /**
     * @brief deleteMapSlot
     * @param id
     * @param robotName
     * Called when we want to stop the recovery and delete the map of a robot
     */
    void deleteMapSlot(int id, QString robotName);

    /**
     * @brief refreshIds
     * Refresh the ids of the ScanMapListItemWidget according to their position in the list
     */
    void refreshIds();

signals:
    void teleopCmd(QString, int);
    void robotGoTo(QString, double, double);
    void stopRecoveringRobots(QStringList);
    /**
     * @brief startRecovering
     * To Start from the beggining to recover a robot's position
     */
    void startRecovering(QString);
    /**
     * @brief playRecovery
     * To play/pause a position recovery that is already launched
     */
    void playRecovery(bool, QString);

protected:

    /**
     * @brief keyPressEvent
     * @param event
     * Called when any key is pressed
     */
    void keyPressEvent(QKeyEvent *event);

    /**
     * @brief closeEvent
     * @param event
     * Called when we close the widget
     */
    void closeEvent(QCloseEvent *event);

private:
    QSharedPointer<Robots> robots;

    QHBoxLayout* mainLayout;

    MergeMapListWidget* listWidget;

    CustomQGraphicsView* graphicsView;
    QGraphicsScene* scene;

    QSize mapSize;
};

#endif /// ROBOTPOSITIONRECOVERY_H
