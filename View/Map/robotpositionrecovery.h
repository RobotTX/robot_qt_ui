#ifndef ROBOTPOSITIONRECOVERY_H
#define ROBOTPOSITIONRECOVERY_H

class Robots;
class QHBoxLayout;
class MergeMapListWidget;
class QGraphicsScene;
class CustomQGraphicsView;

#include <QSharedPointer>
#include <QWidget>

class RobotPositionRecovery : public QWidget {

    Q_OBJECT

public:
    RobotPositionRecovery(QSharedPointer<Robots> _robots, QWidget* parent = Q_NULLPTR);

    void initializeMenu(void);
    void initializeMap(void);

private slots:
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

    void recoverRobotPosSlot(void);

    void startRecoveringSlot(void);

signals:
    void teleopCmd(QString, int);


private:
    QSharedPointer<Robots> robots;

    QHBoxLayout* mainLayout;

    MergeMapListWidget* listWidget;

    CustomQGraphicsView* graphicsView;
    QGraphicsScene* scene;
};

#endif /// ROBOTPOSITIONRECOVERY_H
