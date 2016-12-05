#ifndef TOPLAYOUT_H
#define TOPLAYOUT_H

class QHBoxLayout;
class CustomPushButton;

#include <QWidget>
#include <QMainWindow>
#include <QLabel>

class TopLayout : public QWidget{
    Q_OBJECT

public:

    TopLayout(QMainWindow* parent);

    QString getLabelText(void) const { return label->text(); }
    QString getLabelPerm(void) const { return labelPerm->text(); }


public:
    /**
     * @brief setLabel
     * @param msgType
     * @param msg
     * sets message at the top of the application
     */
    void setLabel(const QString msgType, const QString msg);

    /**
     * @brief setLabelPerm
     * @param msgType
     * @param msg
     * sets the bottom message at the top of the application
     */
    void setLabelPerm(const QString msgType, const QString msg);

    /**
     * @brief setEnable
     * @param enable
     * enables or disables the buttons at the top of the application
     */
    void setEnable(const bool enable);

    /**
     * @brief setLabelDelay
     * @param msgType
     * @param msg
     * @param delayTime
     * sets a message with a delay
     */
    void setLabelDelay(const QString msgType, const QString msg, int delayTime);

    /**
     * @brief delay
     * @param ms
     * introduces a delay of <ms> milliseconds
     */
    void delay(const int ms);

    void addRobotWithoutHome(QString robotName);
    void removeRobotWithoutHome(QString robotName);
    void setRobotNoHomeLabel();

private:
    QHBoxLayout* layout;
    QLabel* label;
    QLabel* labelPerm;
    CustomPushButton* menuBtn;
    CustomPushButton* closeBtn;
    CustomPushButton* centerBtn;
    CustomPushButton* settingBtn;
    CustomPushButton* saveMapBtn;
    CustomPushButton* testButton;
    QVector<QString> robotsWithoutHome;
};

#endif // TOPLAYOUT_H
