#include "pathpointlist.h"
#include "View/pathpointcreationwidget.h"
#include <QDebug>
#include <QModelIndex>

PathPointList::PathPointList(QWidget* parent):QListWidget(parent){
    setDragDropMode(QAbstractItemView::InternalMove);
    setFrameShape(QFrame::NoFrame);
    viewport()->setAutoFillBackground( false );
    //setStyleSheet("QListWidget::item:hover {background-color:grey;}");
    setStyleSheet(" QListWidget {color: red;};\
                    QListWidget::item:hover {background-color:grey;}");

    connect(model(), SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)), this, SLOT(itemMoved(QModelIndex,int,int,QModelIndex,int)));
}

void PathPointList::itemMoved(QModelIndex parent, int first, int last,
                                 QModelIndex destination, int row){

    qDebug() << "itemMoved from" << first << "to" << row;
    if(row >= 0){
        int itemNb = row;
        if(itemNb >= count())
            itemNb = count()-1;
        qDebug() << "item " << ((PathPointCreationWidget*) itemWidget(item(itemNb)))->getName() << "moved to row" << row;
        for(int i = 0; i < count(); i++){
            ((PathPointCreationWidget*) itemWidget(item(i)))->setId(i+1);
            if(i < count()-1){
                ((PathPointCreationWidget*) itemWidget(item(i)))->displayActionWidget(true);
            } else {
                ((PathPointCreationWidget*) itemWidget(item(i)))->displayActionWidget(false);
                ((PathPointCreationWidget*) itemWidget(item(i)))->resetAction();
            }
        }
    }

    emit itemMovedSignal(first, row);
}

void PathPointList::refresh(void){
    for(int i = 0; i < count(); i++){
        ((PathPointCreationWidget*) itemWidget(item(i)))->setId(i+1);
        if(i < count()-1){
            ((PathPointCreationWidget*) itemWidget(item(i)))->displayActionWidget(true);
        } else {
            ((PathPointCreationWidget*) itemWidget(item(i)))->displayActionWidget(false);
            ((PathPointCreationWidget*) itemWidget(item(i)))->resetAction();
        }
    }
}
