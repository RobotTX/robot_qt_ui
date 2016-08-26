#ifndef CUSTOMPUSHBUTTON_H
#define CUSTOMPUSHBUTTON_H

class QMouseEvent;
class QLabel;

#include <QPushButton>


class CustomPushButton: public QPushButton{
    Q_OBJECT
public:
    enum ButtonType { TOP, BOTTOM, LEFT_MENU };
    CustomPushButton(const QIcon& icon, const QString &text = "", QWidget *parent = Q_NULLPTR, const ButtonType type = LEFT_MENU,
                     const bool checkable = false, const bool enable = true);
    CustomPushButton(const QString &text = "", QWidget *parent = Q_NULLPTR, const ButtonType type = LEFT_MENU,
                     const bool checkable = false, const bool enable = true);
    void initialize(const bool checkable, const bool enable);
    void addStyleSheet(const QString style);
    void setText(const QString &text);
    void moveLabel();

private:
    ButtonType buttonType;
    QLabel* label;

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);
    void enterEvent(QEvent *e);
    void resizeEvent(QResizeEvent *event);
    void showEvent(QShowEvent* event);

signals:
    void doubleClick(QString);
};

#endif // CUSTOMPUSHBUTTON_H
