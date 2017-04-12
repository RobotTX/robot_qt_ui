#ifndef SCANMAPPAINTEDITEM_H
#define SCANMAPPAINTEDITEM_H

#include <QtQuick/QQuickPaintedItem>
#include <QImage>


class ScanMapPaintedItem : public QQuickPaintedItem {

        Q_OBJECT

    Q_PROPERTY(float orientationRobot READ robotOrientation WRITE setRobotOrientation NOTIFY orientationRobotChanged)
    Q_PROPERTY(float xRobot READ robotX WRITE setRobotX NOTIFY xRobotChanged)
    Q_PROPERTY(float yRobot READ robotY WRITE setRobotY NOTIFY yRobotChanged)

public:

    ScanMapPaintedItem(QQuickItem* parent = 0);

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

    void rotate(const int angle);

    void setImage(const QPair<QImage, QPoint> image_and_shift) {
        _image = image_and_shift.first;
        left = image_and_shift.second.x();
        top = image_and_shift.second.y();
    }

    QImage& getImage(void) { return _image; }

    float robotOrientation(void) const { return orientationRobot; }
    float robotX(void) const { return xRobot; }
    float robotY(void) const { return yRobot; }

    void setRobotX(const float x) { xRobot = x - left; }
    void setRobotY(const float y) { yRobot = y - top; }
    void setRobotOrientation(const float ori) { orientationRobot = ori; emit updateRobot(); }

signals:

    void orientationRobotChanged();
    void xRobotChanged();
    void yRobotChanged();
    void updateRobot();

private:
    int left;
    int top;
    float xRobot;
    float yRobot;
    float orientationRobot;
    QImage _image;

};

#endif /// SCANMAPPAINTEDITEM_H
