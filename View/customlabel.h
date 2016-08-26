#ifndef CUSTOMLABEL_H
#define CUSTOMLABEL_H

#include <QLabel>

class CustomLabel : public QLabel {
public:
    CustomLabel(QWidget *parent = Q_NULLPTR);
    CustomLabel(const QString &text, QWidget *parent = Q_NULLPTR);

protected:
    void resizeEvent(QResizeEvent *event);

};

#endif // CUSTOMLABEL_H
