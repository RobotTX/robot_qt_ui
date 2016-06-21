#ifndef TOPLAYOUT_H
#define TOPLAYOUT_H

class QHBoxLayout;
class QMainWindow;
class QLabel;

#include <QWidget>

#define TEXT_COLOR_NORMAL "#333333"
#define TEXT_COLOR_INFO "#31708f"
#define TEXT_COLOR_SUCCESS "#3c763d"
#define TEXT_COLOR_WARNING "#8a6d3b"
#define TEXT_COLOR_DANGER "#a94442"

class TopLayout : public QWidget{
public:
    TopLayout(QMainWindow* parent);
    ~TopLayout();
    void setLabel(QString msgType, QString msg);

private:
    QHBoxLayout* layout;
    QLabel* label;
};

#endif // TOPLAYOUT_H
