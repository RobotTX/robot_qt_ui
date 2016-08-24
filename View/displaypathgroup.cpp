#include "displaypathgroup.h"
#include "Model/paths.h"
#include "View/pathbuttongroup.h"
#include "View/customscrollarea.h"
#include "View/topleftmenu.h"
#include <QVBoxLayout>
#include <QMainWindow>

DisplayPathGroup::DisplayPathGroup(QMainWindow* _parent, const QSharedPointer<Paths>& _paths):
    QWidget(_parent), paths(_paths)
{
    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);

    actionButtons->disableAll();
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(false);

    actionButtons->getPlusButton()->setToolTip("To add a point click on the map");
    actionButtons->getMinusButton()->setToolTip("Select a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a point and click here to modify it");
    actionButtons->getGoButton()->setToolTip("Select a point and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a point and click here to display or hide it on the map");

    layout->addWidget(actionButtons);
    // if it crashes change the parent of this guy
    pathButtonGroup = new PathButtonGroup(_parent, paths);


    //connect(pointButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(buttonClickedSlot(QAbstractButton*)));
    scrollArea->setWidget(pathButtonGroup);

    layout->addWidget(scrollArea);
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setContentsMargins(0,0,0,0);

}
