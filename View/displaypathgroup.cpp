#include "displaypathgroup.h"
#include "Model/paths.h"
#include "View/pathbuttongroup.h"
#include "View/customscrollarea.h"
#include "View/topleftmenu.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include "View/custompushbutton.h"

DisplayPathGroup::DisplayPathGroup(QWidget* _parent, const QSharedPointer<Paths>& _paths):
    QWidget(_parent), paths(_paths), lastCheckedButton("")
{
    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);

    initializeActionButtons();

    layout->addWidget(actionButtons);

    pathButtonGroup = new PathButtonGroup(this, paths);
    connect(pathButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));

    scrollArea->setWidget(pathButtonGroup);

    layout->addWidget(scrollArea);
    /*setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);*/
    //layout->setContentsMargins(0,0,0,0);
}

void DisplayPathGroup::showEvent(QShowEvent *event){
    initializeActionButtons();
    QWidget::showEvent(event);
}

void DisplayPathGroup::initializeActionButtons(){
    actionButtons->disableAll();
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(true);

    actionButtons->getPlusButton()->setToolTip("Click to create a new path");
    actionButtons->getMinusButton()->setToolTip("Select a path and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a path and click here to edit it");
    actionButtons->getGoButton()->setToolTip("Select a path and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a path and click here to display or hide it on the map");
}

void DisplayPathGroup::enableButtons(QAbstractButton *button){
    if(button->text().compare(lastCheckedButton)){
        lastCheckedButton = button->text();
        actionButtons->getMinusButton()->setEnabled(true);
        actionButtons->getMinusButton()->setToolTip("Click to remove the selected path");
        actionButtons->getMapButton()->setEnabled(true);
        actionButtons->getMapButton()->setToolTip("Click to display or hide the selected path on the map");
        actionButtons->getEditButton()->setEnabled(true);
        actionButtons->getEditButton()->setToolTip("Click to edit the selected path");
        actionButtons->getGoButton()->setEnabled(true);
        actionButtons->getGoButton()->setToolTip("Click to access the information of the selected path");
    } else { /// the path was already selected so we deselect it
        lastCheckedButton = "";
        initializeActionButtons();
        pathButtonGroup->uncheck();
    }
}
