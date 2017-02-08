#ifndef ROBOTPOSITIONRECOVERY_H
#define ROBOTPOSITIONRECOVERY_H

class Robots;
class QHBoxLayout;

#include <QSharedPointer>
#include <QWidget>

class robotPositionRecovery : public QWidget {

    Q_OBJECT

public:
    robotPositionRecovery(QSharedPointer<Robots> _robots, QWidget* parent = Q_NULLPTR);

    void initializeMenu(void);

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
    void teleopCmdSlot(int);


private:
    QSharedPointer<Robots> robots;

    QHBoxLayout* mainLayout;
};

#endif /// ROBOTPOSITIONRECOVERY_H
