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
    void disable();
    void enable();
    void setLabelDelay(const QString msgType, const QString msg, int delayTime);
    void delay(const int ms) ;


private:
    QHBoxLayout* layout;
    QLabel* label;
    QPushButton* menuBtn;
    QPushButton* connectBtn;
    QPushButton* closeBtn;
    QPushButton* centerBtn;
};

#endif // TOPLAYOUT_H
