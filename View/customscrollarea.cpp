#include "customscrollarea.h"
#include <QEvent>
#include <QScrollBar>
#include <QDebug>

CustomScrollArea::CustomScrollArea(QWidget *parent, bool _vertical, QScrollBar* _childBar):
    QScrollArea(parent), vertical(_vertical), childBar(_childBar){
    setWidgetResizable(true);
    if(vertical){
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    } else {
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    }
    setFrameShape(QFrame::NoFrame);
}

bool CustomScrollArea::eventFilter(QObject *o, QEvent *e){
    if(o && o == widget() && e->type() == QEvent::Resize){
        if(vertical){
            setMinimumWidth(widget()->minimumSizeHint().width() + verticalScrollBar()->width());
        } else {
            setMinimumHeight(widget()->minimumSizeHint().height() + horizontalScrollBar()->height());
        }
    }

    if(childBar != NULL && e->type() == QEvent::Resize){
        if(vertical){
            childBar->setMinimum(verticalScrollBar()->minimum());
            childBar->setMaximum(verticalScrollBar()->maximum());
            childBar->setValue(verticalScrollBar()->value());
            childBar->setPageStep(verticalScrollBar()->pageStep());
            childBar->resize(15, height());
        } else {
            childBar->setMinimum(horizontalScrollBar()->minimum());
            childBar->setMaximum(horizontalScrollBar()->maximum());
            childBar->setValue(horizontalScrollBar()->value());
            childBar->setPageStep(horizontalScrollBar()->pageStep());
            childBar->resize(width(), 15);
        }
        if(childBar->maximum() <= 0){
            
            childBar->hide();
        } else {
            childBar->show();
        }
    }

    return QScrollArea::eventFilter(o, e);
}
