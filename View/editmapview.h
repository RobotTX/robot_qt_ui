#ifndef EDITMAPVIEW_H
#define EDITMAPVIEW_H

#include <QObject>
#include <QGraphicsPixmapItem>

class EditMapView: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT
public:
    EditMapView(const QPixmap& pixmap);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    QPointF dragStartPosition;
};

#endif // EDITMAPVIEW_H
