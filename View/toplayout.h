#ifndef TOPLAYOUT_H
#define TOPLAYOUT_H

class QHBoxLayout;
class QLabel;
class QPushButton;

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
    void setLabel(const QString msgType, const QString msg);
    void setEnable(bool enable);
    void enable();
    void setLabelDelay(const QString msgType, const QString msg, int delayTime);
    void delay(const int ms) ;
    QPair<QString, QString> getLastMessage(void) const { return lastMessage; }
    void setLastMessage(const QString msgType, const QString message) { lastMessage.first = msgType; lastMessage.second = message; }

private:
    QHBoxLayout* layout;
    QLabel* label;
    QPushButton* menuBtn;
    QPushButton* connectBtn;
    QPushButton* closeBtn;
    QPushButton* centerBtn;
    QPair<QString, QString> lastMessage;
};

#endif // TOPLAYOUT_H
