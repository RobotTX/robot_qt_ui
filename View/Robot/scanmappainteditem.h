#ifndef SCANMAPPAINTEDITEM_H
#define SCANMAPPAINTEDITEM_H

#include <QtQuick/QQuickPaintedItem>
#include <QImage>
#include <QMouseEvent>


class ScanMapPaintedItem : public QQuickPaintedItem {

        Q_OBJECT

    Q_PROPERTY(float orientationRobot READ robotOrientation WRITE setRobotOrientation NOTIFY orientationRobotChanged)
    Q_PROPERTY(float xRobot READ robotX WRITE setRobotX NOTIFY xRobotChanged)
    Q_PROPERTY(float yRobot READ robotY WRITE setRobotY NOTIFY yRobotChanged)
    Q_PROPERTY(QString ip READ getIp WRITE setIp NOTIFY ipChanged)
    Q_PROPERTY(bool _drawRobotView READ drawRobotView WRITE setDrawRobotView NOTIFY drawRobotViewChanged)

public:

    ScanMapPaintedItem(QQuickItem* parent = 0);

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

    void setImage(const QPair<QImage, QPoint> image_and_shift) {
        _image = image_and_shift.first;
        left = image_and_shift.second.x();
        top = image_and_shift.second.y();
    }

    QImage& getImage(void) { return _image; }

    float robotOrientation(void) const { return orientationRobot; }
    float robotX(void) const { return xRobot; }
    float robotY(void) const { return yRobot; }
    QString getIp(void) const { return ip; }
    bool drawRobotView(void) const { return _drawRobotView; }
    int getLeft(void) const { return left; }
    int getTop(void) const { return top; }

    void setRobotX(const float x);
    void setRobotY(const float y);

    void setRobotOrientation(const float ori) { orientationRobot = ori; emit updateRobot(); }
    void setIp(const QString _ip) { ip = _ip; }
    void setDrawRobotView(const bool draw) {
                                            _drawRobotView = draw;
                                            if(!_drawRobotView) emit hideRobot();
                            }

signals:

    void orientationRobotChanged();
    void xRobotChanged();
    void yRobotChanged();
    void ipChanged();
    void updateRobot();
    void drawRobotViewChanged();
    void hideRobot();

private:
    /// before saving drawRobotView must be set to false in order to not save it with the map
    bool _drawRobotView;
    QString ip;
    int left;
    int top;
    float xRobot;
    float yRobot;
    float orientationRobot;
    QImage _image;
};

#endif /// SCANMAPPAINTEDITEM_H
