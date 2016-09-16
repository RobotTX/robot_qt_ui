#ifndef TOPLAYOUT_H
#define TOPLAYOUT_H

class QHBoxLayout;
class QLabel;
class CustomPushButton;

#include <QWidget>
#include <QMainWindow>

#define TEXT_COLOR_NORMAL "#333333"
#define TEXT_COLOR_INFO "#31708f"
#define TEXT_COLOR_SUCCESS "#3c763d"
#define TEXT_COLOR_WARNING "#8a6d3b"
#define TEXT_COLOR_DANGER "#a94442"

class TopLayout : public QWidget{
    Q_OBJECT

public:

    TopLayout(QMainWindow* parent);

    QLabel* getLabel(void) const { return label; }
    QPair<QString, QString> getLastMessage(void) const { return lastMessage; }

    void setLastMessage(const QString msgType, const QString message) { lastMessage.first = msgType; lastMessage.second = message; }

public:
    /**
     * @brief setLabel
     * @param msgType
     * @param msg
     * sets message at the top of the application
     */
    void setLabel(const QString msgType, const QString msg);
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

private:
    QHBoxLayout* layout;
    QLabel* label;
    CustomPushButton* menuBtn;
    CustomPushButton* closeBtn;
    CustomPushButton* centerBtn;
    CustomPushButton* settingBtn;
    CustomPushButton* saveMapBtn;
    QPair<QString, QString> lastMessage;
};

#endif // TOPLAYOUT_H
