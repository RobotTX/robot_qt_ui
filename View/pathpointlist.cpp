#include "pathpointlist.h"
#include "View/pathpointcreationwidget.h"
#include <QDebug>
#include <QModelIndex>
#include <QComboBox>
#include <QLineEdit>

PathPointList::PathPointList(QWidget* parent):QListWidget(parent){
    setDragDropMode(QAbstractItemView::InternalMove);
    setFrameShape(QFrame::NoFrame);
    viewport()->setAutoFillBackground( false );
    //setStyleSheet("QListWidget::item:hover {background-color:grey;}");
    setStyleSheet(" QListWidget {color: red;};\
                    QListWidget::item:hover {background-color:grey;}");

    connect(model(), SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)), this, SLOT(itemMoved(QModelIndex,int,int,QModelIndex,int)));
}

void PathPointList::itemMoved(QModelIndex /* unused */, int first, int /* unused */,
                                 QModelIndex /* unused */, int row){

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


void PathPointList::update(int indexNb,int action, int time  ){

    ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getAction()->setCurrentIndex(action);
    if(action==0 && time!=NULL)
    {
        ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getTimeEdit()->setText( QString::number(time));
        ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getTimeWidget()->show();
    }
    else
        ((PathPointCreationWidget*) itemWidget(item(indexNb)))->getTimeWidget()->hide();

}
