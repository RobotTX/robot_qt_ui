#ifndef TOPLAYOUTWIDGET_H
#define TOPLAYOUTWIDGET_H

class QHBoxLayout;
class CustomPushButton;

#include <QWidget>
#include <QMainWindow>
#include <QLabel>
#include "Model/toplayout.h"

class TopLayoutWidget : public QWidget{
    Q_OBJECT

public:

    TopLayoutWidget(const TopLayout &topLayout, QMainWindow* parent = 0);

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
    void setRobotNoHomeLabel(const QString robots_string);

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
};

#endif // TOPLAYOUT_H
