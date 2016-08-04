#ifndef TOPLEFTMENU_H
#define TOPLEFTMENU_H

class SpaceWidget;
#include <QWidget>
class QVBoxLayout;
class QPushButton;
class QHBoxLayout;

#include <QPushButton>
#include <buttonmenu.h>

class TopLeftMenu : public QWidget{
    Q_OBJECT
public:
    TopLeftMenu(QWidget* parent);
    //getters
    ButtonMenu* getPlusButton(void) const {return plusButton;}
    QPushButton* getMinusButton(void) const {return minusButton;}
    ButtonMenu* getEditButton(void) const {return editButton;}
    QPushButton* getMapButton(void) const {return mapButton;}
    QPushButton* getGoButton(void) const {return goButton;}


    void disableAll();
    void enableAll();
    void uncheckAll();
    void checkAll();
    void setAllCheckable();
    void setAllNonCheckable();
    void setEnable(bool enable);


private:
   QVBoxLayout* layout ;
   QHBoxLayout* grid;

   ButtonMenu* plusButton;
   QPushButton* minusButton;
   ButtonMenu* editButton;
   QPushButton* mapButton;
   QPushButton* goButton;
   QHBoxLayout* eyeMapLayout;
   SpaceWidget* spaceWidget;
   QList<QPushButton*> enabledBtns;
};


#endif // TOPLEFTMENU_H
