#ifndef MERGEMAPSPAINTEDITEM_H
#define MERGEMAPSPAINTEDITEM_H

#include <QtQuick/QQuickPaintedItem>
#include <QImage>

class QMouseEvent;

class MergeMapsPaintedItem : public QQuickPaintedItem {

    Q_OBJECT

public:

    MergeMapsPaintedItem(QQuickItem* parent = 0);

    void rotate(const int angle);
    int getAngle(void) const { return _angle; }

    QImage& getImage(void) { return _image; }

    void setImage(const QImage image) {
        qDebug() << "width and height" << image.width() << image.height();
        _image = image;
    }

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

private:
    int _angle;
    QImage _image;
};

#endif /// MERGEMAPSPAINTEDITEM_H
