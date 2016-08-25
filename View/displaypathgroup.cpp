#include "displaypathgroup.h"
#include "Model/paths.h"
#include "View/pathbuttongroup.h"
#include "View/customscrollarea.h"
#include "View/topleftmenu.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include "View/custompushbutton.h"

DisplayPathGroup::DisplayPathGroup(QWidget* _parent, const QSharedPointer<Paths>& _paths):
    QWidget(_parent), paths(_paths)
{
    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);

    initializeActionButtons();

    layout->addWidget(actionButtons);

    pathButtonGroup = new PathButtonGroup(this, paths);

    //connect(pointButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(buttonClickedSlot(QAbstractButton*)));
    scrollArea->setWidget(pathButtonGroup);

    layout->addWidget(scrollArea);
    /*setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);*/
    //layout->setContentsMargins(0,0,0,0);
}

void DisplayPathGroup::showEvent(QEvent *event){

}

void DisplayPathGroup::initializeActionButtons(){
    actionButtons->disableAll();
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(false);

    actionButtons->getPlusButton()->setToolTip("Click to create a new path");
    actionButtons->getMinusButton()->setToolTip("Select a path and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a path and click here to edit it");
    actionButtons->getGoButton()->setToolTip("Select a path and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a path and click here to display or hide it on the map");
}
