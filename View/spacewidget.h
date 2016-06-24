#ifndef SPACEWIDGET_H
#define SPACEWIDGET_H

#include <QWidget>
class QFrame;

/**
 * @brief The SpaceWidget class
 * Draw a vertical or horizontal line
 */
class SpaceWidget : public QWidget{
public:
    enum SpaceOrientation {VERTICAL, HORIZONTAL};
    SpaceWidget(const SpaceOrientation orientation, QWidget *parent);

    /**
     * @brief setColor
     * @param color
     * To change the color of the line, can use any HTML color recognized by the stylesheet
     */
    void setColor(const QString color);

private:
    QFrame* spaceFrame;
};

#endif // SPACEWIDGET_H
