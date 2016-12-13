#include "mergemaplistwidget.h"
#include "stylesettings.h"
#include <QKeyEvent>

MergeMapListWidget::MergeMapListWidget(QWidget *parent) : QListWidget(parent){

    //setDragDropMode(QAbstractItemView::InternalMove);
    setFrameShape(QFrame::NoFrame);
    viewport()->setAutoFillBackground(false);
    setAttribute(Qt::WA_MacShowFocusRect, false);
    setStyleSheet(" QListWidget {color: red;}\
                  QListWidget::item {border-bottom: 1px solid; border-bottom-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #949494, stop: 1 transparent); }\
                      QListWidget::item:hover {background-color:"+button_hover_color+";}\
                      QListWidget::item:selected {background-color:"+button_checked_color+";}");
}


void MergeMapListWidget::keyPressEvent(QKeyEvent* event){
    if((event->key() == Qt::Key_Up || event->key() == Qt::Key_Down
                         || event->key() == Qt::Key_Left || event->key() == Qt::Key_Right))
            emit dirKeyPressed(event->key());
    else
        QListWidget::keyPressEvent(event);
}
