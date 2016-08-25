#include "pathpointlist.h"
#include "View/pathpointcreationwidget.h"
#include <QDebug>
#include <QModelIndex>
#include <QComboBox>
#include <QLineEdit>
#include "stylesettings.h"

PathPointList::PathPointList(QWidget* parent): QListWidget(parent){
    setDragDropMode(QAbstractItemView::InternalMove);
    setFrameShape(QFrame::NoFrame);
    viewport()->setAutoFillBackground( false );
    setAttribute(Qt::WA_MacShowFocusRect, false);
    setStyleSheet(" QListWidget {color: red;  }\
                  QListWidget::item {border-bottom: 1px solid; border-bottom-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #949494, stop: 1 transparent); }\
                      QListWidget::item:hover {background-color:"+button_hover_color+";}\
                      QListWidget::item:selected {background-color:"+button_checked_color+";}");


    connect(model(), SIGNAL(rowsMoved(QModelIndex, int, int, QModelIndex, int)), this, SLOT(itemMoved(QModelIndex, int, int, QModelIndex, int)));
}

void PathPointList::itemMoved(QModelIndex parent, int first, int end,
                                 QModelIndex destination, int row){

    qDebug() << "itemMoved from" << first << "to" << row;
    if(row >= 0){
        int itemNb = row;
        if(itemNb >= count())
            itemNb = count()-1;
        qDebug() << "item " << ((PathPointCreationWidget*) itemWidget(item(itemNb)))->getName() << "moved to row" << row;
        refresh();
    }

    emit itemMovedSignal(parent, first, end, destination, row);
}

void PathPointList::refresh(void){
    for(int i = 0; i < count(); i++){
        static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->setId(i);
        if(i < count()-1){
            static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->displayActionWidget(true);
        } else {
            static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->displayActionWidget(false);
            static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->resetAction();
        }
    }
}


void PathPointList::update(const int indexNb, const int action, const int time){

    ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getAction()->setCurrentIndex(action);
    if(action==0 && time != 0)
    {
        ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getTimeEdit()->setText( QString::number(time));
        ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getTimeWidget()->show();
    }
    else
        ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getTimeWidget()->hide();

}
