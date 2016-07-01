#ifndef TOPLEFTMENU_H
#define TOPLEFTMENU_H

#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>

class TopLeftMenu : public QWidget{
    Q_OBJECT
public:
    TopLeftMenu(QWidget* parent);
    //getters
    QPushButton* getPlusButton(void) const {return plusButton;}
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

   QPushButton* plusButton;
   QPushButton* minusButton;
   QPushButton* editButton;
   QPushButton* mapButton;
   QPushButton* eyeButton;
   QHBoxLayout* eyeMapLayout;

};


#endif // TOPLEFTMENU_H
