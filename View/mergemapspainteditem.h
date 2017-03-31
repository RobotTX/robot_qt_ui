#ifndef MERGEMAPSPAINTEDITEM_H
#define MERGEMAPSPAINTEDITEM_H

#include <QtQuick/QQuickPaintedItem>
#include <QImage>
#include <QPointer>

class QMouseEvent;

class MergeMapsPaintedItem : public QQuickPaintedItem {

    Q_OBJECT

    Q_PROPERTY(int _width READ image_width WRITE set_width NOTIFY widthChanged)
    Q_PROPERTY(int _height READ image_height WRITE set_height NOTIFY heightChanged)

public:

    MergeMapsPaintedItem(QQuickItem* parent = 0);

    int image_width() const { return _width; }
    int image_height() const { return _height; }

    void set_width(const int width) { _width = width; }
    void set_height(const int height) { _height = height; }

    void rotate(const int angle);

    void setImage(const QImage image) {
        qDebug() << "width and height" << image.width() << image.height();
        _image = image;
        _width = image.width();
        _height = image.height();
        emit widthChanged();
        emit heightChanged();
    }

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

signals:
    void widthChanged();
    void heightChanged();

private:
    QImage _image;
    int _width;
    int _height;
};

#endif /// MERGEMAPSPAINTEDITEM_H
