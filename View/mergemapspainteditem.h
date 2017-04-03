#ifndef MERGEMAPSPAINTEDITEM_H
#define MERGEMAPSPAINTEDITEM_H

#include <QtQuick/QQuickPaintedItem>
#include <QImage>
#include <QPointer>

class QMouseEvent;

class MergeMapsPaintedItem : public QQuickPaintedItem {

    Q_OBJECT

public:

    MergeMapsPaintedItem(QQuickItem* parent = 0);

    void rotate(const int angle);

    void setImage(const QImage image) {
        qDebug() << "width and height" << image.width() << image.height();
        _image = image;
    }

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

signals:

    void widthChanged();
    void heightChanged();

private:

    QImage _image;
};

#endif /// MERGEMAPSPAINTEDITEM_H
