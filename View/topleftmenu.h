#ifndef TOPLEFTMENU_H
#define TOPLEFTMENU_H

class SpaceWidget;
#include <QWidget>
class QVBoxLayout;
class QPushButton;
class QHBoxLayout;
class ButtonMenu;

class TopLeftMenu : public QWidget{
    Q_OBJECT
public:
    TopLeftMenu(QWidget* parent);
    //getters
    ButtonMenu* getPlusButton(void) const {return plusButton;}
    QPushButton* getMinusButton(void) const {return minusButton;}
    QPushButton* getEditButton(void) const {return editButton;}
    QPushButton* getMapButton(void) const {return mapButton;}
    QPushButton* getEyeButton(void) const {return eyeButton;}


    void disableAll();
    void enableAll();
    void uncheckAll();
    void checkAll();
    void setAllCheckable();
    void setAllNonCheckable();


private:
   QVBoxLayout* layout ;
   QHBoxLayout* grid;

   ButtonMenu* plusButton;
   QPushButton* minusButton;
   QPushButton* editButton;
   QPushButton* mapButton;
   QPushButton* eyeButton;
   QHBoxLayout* eyeMapLayout;
   SpaceWidget* spaceWidget;
};


#endif // TOPLEFTMENU_H
