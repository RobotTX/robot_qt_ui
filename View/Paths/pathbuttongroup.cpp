#include "pathbuttongroup.h"
#include <QVBoxLayout>
#include "Model/Paths/paths.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/stylesettings.h"
#include <QResizeEvent>

PathButtonGroup::PathButtonGroup(QWidget *_parent): QWidget(_parent)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 10, 0);
}

/// delete the buttons so they can be reconstructed (to update the QButtonGroup in the event of a creating or edition)
void PathButtonGroup::deleteButtons(void){
    while(QLayoutItem* item = layout->takeAt(0)){
        if(QWidget* button = item->widget()){
            delete button;
            button = 0;
        }
        delete item;
        item = 0;
    }
}

void PathButtonGroup::uncheck(){
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

void PathButtonGroup::setCheckable(const bool checkable){
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setCheckable(checkable);
}

void PathButtonGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent()->parent()->parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
