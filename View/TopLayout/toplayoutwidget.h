#ifndef TOPLAYOUTWIDGET_H
#define TOPLAYOUTWIDGET_H

class QHBoxLayout;
class CustomPushButton;
class MainWindow;

#include <QWidget>
#include <QLabel>

class TopLayoutWidget : public QWidget{
    Q_OBJECT

public:

    TopLayoutWidget(MainWindow* parent = 0);

    QString getLabelText(void) const { return label->text(); }
    QString getLabelPerm(void) const { return labelPerm->text(); }
    CustomPushButton* getSaveButton(void) const { return saveMapBtn; }

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
    void setLabelDelay(const QString msgType, const QString msg, const int delayTime);

    void setRobotNoHomeLabel(const QString robots_string);

private:
    CustomPushButton* menuBtn;
    CustomPushButton* closeBtn;
    CustomPushButton* centerBtn;
    CustomPushButton* settingBtn;
    CustomPushButton* saveMapBtn;
    CustomPushButton* testButton;
    QLabel* label;
    QLabel* labelPerm;
};

#endif // TOPLAYOUT_H
