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
    QPushButton* getPlusButton() {return plusButton;}
    QPushButton* getMinusButton() {return minusButton;}
    QPushButton* getEditButton() {return editButton;}
    QPushButton* getMapButton() {return mapButton;}
    QPushButton* getEyeButton() {return eyeButton;}

    //setters
    void setPlusButton(QPushButton* plus) { plusButton = plus;}
    void setMinusButton(QPushButton* minus) { minusButton = minus;}
    void setEditButton(QPushButton* edit) { editButton = edit;}
    void setMapButton(QPushButton* map) { mapButton = map;}
    void setEyeButton(QPushButton* eye) { eyeButton = eye;}

    void disableAll();
    void EnableAll();
    void uncheckAll();
    void checkAll();
    void allCheckable();
    void allNonCheckable();


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
