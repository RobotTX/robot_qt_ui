#include "verticalscrollarea.h"
#include <QEvent>
#include <QScrollBar>

VerticalScrollArea::VerticalScrollArea(QWidget *parent): QScrollArea(parent){
    setWidgetResizable(true);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setFrameShape(QFrame::NoFrame);
}

bool VerticalScrollArea::eventFilter(QObject *o, QEvent *e){
    if(o && o == widget() && e->type() == QEvent::Resize)
        setMinimumWidth(widget()->minimumSizeHint().width() + verticalScrollBar()->width());

    return QScrollArea::eventFilter(o, e);
}
