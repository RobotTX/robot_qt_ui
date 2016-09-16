#include "pathbuttongroup.h"
#include <QVBoxLayout>
#include "Model/paths.h"
#include "View/custompushbutton.h"
#include "View/stylesettings.h"

PathButtonGroup::PathButtonGroup(QWidget *_parent, const QSharedPointer<Paths> &_paths): QWidget(_parent), paths(_paths)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
}

/// delete the buttons so they can be reconstructed (to update the QButtonGroup in the event of a creating or edition)
void PathButtonGroup::deleteButtons(void){
    //qDebug() << "PathButtonGroup::deleteButtons called";
    while(QLayoutItem* item = layout->takeAt(0)){
        if(QWidget* button = item->widget())
            delete button;
    }
}

void PathButtonGroup::uncheck(){
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

void PathButtonGroup::setCheckable(const bool checkable){
    //qDebug() << "GroupButtonGroup::setEnabled called";
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setCheckable(checkable);
}
