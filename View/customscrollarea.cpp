#include "customscrollarea.h"
#include <QEvent>
#include <QScrollBar>
#include <QDebug>
#include <QLayout>
#include <QResizeEvent>

CustomScrollArea::CustomScrollArea(QWidget *parent, bool _leftMenu, bool _editRobotWidget, bool _vertical, QScrollBar* _childBar, QWidget* _hidingWidget):
    QScrollArea(parent), childBar(_childBar), hidingWidget(_hidingWidget), vertical(_vertical), leftMenu(_leftMenu), editRobotWidget(_editRobotWidget){
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
        if(vertical)
            setMinimumWidth(widget()->minimumSizeHint().width() + verticalScrollBar()->width());
        else
            setMinimumHeight(widget()->minimumSizeHint().height() + horizontalScrollBar()->height());
    }

    if(childBar != NULL && e->type() == QEvent::Resize){
        childBar->setMinimum(horizontalScrollBar()->minimum());
        childBar->setMaximum(horizontalScrollBar()->maximum());
        childBar->setValue(horizontalScrollBar()->value());
        childBar->setPageStep(horizontalScrollBar()->pageStep());
        childBar->resize(width(), 15);
        hidingWidget->resize(width(), 15);

        if(childBar->maximum() <= 0){
            childBar->hide();
            hidingWidget->hide();
        } else {
            childBar->show();
            hidingWidget->show();
        }
    }

    return QScrollArea::eventFilter(o, e);
}

void CustomScrollArea::resizeEvent(QResizeEvent *event){
    if(leftMenu){
        QWidget* parentWidget = static_cast<QWidget*>(parent());
        int maxWidth = parentWidget->width();
        setFixedWidth(maxWidth);
        if(editRobotWidget)
            widget()->setMaximumWidth(maxWidth);

    }
    QScrollArea::resizeEvent(event);
}

