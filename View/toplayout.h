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

    QPair<QString, QString> getLastMessage(void) const { return lastMessage; }
    void setLastMessage(const QString msgType, const QString message) { lastMessage.first = msgType; lastMessage.second = message; }

    void setLabel(const QString msgType, const QString msg);
    void setEnable(const bool enable);
    void enable();
    void setLabelDelay(const QString msgType, const QString msg, int delayTime);
    void delay(const int ms);

private:
    QHBoxLayout* layout;
    QLabel* label;
    CustomPushButton* menuBtn;
    CustomPushButton* connectBtn;
    CustomPushButton* closeBtn;
    CustomPushButton* centerBtn;
    CustomPushButton* settingBtn;
    CustomPushButton* saveMapBtn;

    QPair<QString, QString> lastMessage;
};

#endif // TOPLAYOUT_H
