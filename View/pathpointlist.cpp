#include "pathpointlist.h"
#include "View/pathpointcreationwidget.h"
#include <QDebug>
#include <QModelIndex>
#include <QComboBox>
#include <QLineEdit>

PathPointList::PathPointList(QWidget* parent): QListWidget(parent){
    setDragDropMode(QAbstractItemView::InternalMove);
    setFrameShape(QFrame::NoFrame);
    viewport()->setAutoFillBackground( false );
    setStyleSheet(" QListWidget {color: red;};\
                    QListWidget::item:hover {background-color:grey;}");

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
        for(int i = 0; i < count(); i++){
            static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->setId(i+1);
            if(i < count()-1){
                static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->displayActionWidget(true);
            } else {
                static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->displayActionWidget(false);
                static_cast<PathPointCreationWidget*> (itemWidget(item(i)))->resetAction();
            }
        }
    }

    emit itemMovedSignal(parent, first, end, destination, row);
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
