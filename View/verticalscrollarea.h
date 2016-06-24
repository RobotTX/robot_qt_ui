#ifndef VERTICALSCROLLAREA_H
#define VERTICALSCROLLAREA_H

class QEvent;

#include <QScrollArea>


/**
 * @brief The VerticalScrollArea class
 * Custom QScrollArea which only displays the vertical scrollBar
 */
class VerticalScrollArea : public QScrollArea {
    Q_OBJECT

public:
    explicit VerticalScrollArea(QWidget *parent);

    virtual bool eventFilter(QObject *o, QEvent *e);

};

#endif // VERTICALSCROLLAREA_H
