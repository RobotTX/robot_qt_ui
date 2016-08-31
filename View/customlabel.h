#ifndef CUSTOMLABEL_H
#define CUSTOMLABEL_H

#include <QLabel>

class CustomLabel : public QLabel {
public:
    CustomLabel(QWidget *parent = Q_NULLPTR, bool title = false);
    CustomLabel(const QString &text, QWidget *parent = Q_NULLPTR, bool title = false);
    void moveLabel();
    void initialize();

protected:
    void resizeEvent(QResizeEvent *event);
    void enterEvent(QEvent *event);

private:
    QLabel* label;
    bool title;

};

#endif // CUSTOMLABEL_H
