#ifndef SPACEWIDGET_H
#define SPACEWIDGET_H

#include <QWidget>
class QFrame;

class SpaceWidget : public QWidget{
public:
    enum SpaceOrientation {VERTICAL, HORIZONTAL};
    SpaceWidget(SpaceOrientation orientation);
    void setColor(QString color);

private:
    QFrame* spaceFrame;
};

#endif // SPACEWIDGET_H
