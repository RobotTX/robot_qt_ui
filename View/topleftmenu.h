#ifndef TOPLEFTMENU_H
#define TOPLEFTMENU_H

class SpaceWidget;
class QVBoxLayout;
class CustomPushButton;
class QHBoxLayout;
class CustomPushButton;

#include <QWidget>

class TopLeftMenu : public QWidget{
    Q_OBJECT

public:

    TopLeftMenu(QWidget* parent);

    CustomPushButton* getPlusButton(void) const {return plusButton;}
    CustomPushButton* getMinusButton(void) const {return minusButton;}
    CustomPushButton* getEditButton(void) const {return editButton;}
    CustomPushButton* getMapButton(void) const {return mapButton;}
    CustomPushButton* getGoButton(void) const {return goButton;}

    void enableAll(const bool enable);
    void checkAll(const bool check);
    void setCheckable(const bool checkable);
    void setEnable(const bool enable);

private:
   QVBoxLayout* layout ;
   QHBoxLayout* grid;

   CustomPushButton* plusButton;
   CustomPushButton* minusButton;
   CustomPushButton* editButton;
   CustomPushButton* mapButton;
   CustomPushButton* goButton;
   QHBoxLayout* eyeMapLayout;
   SpaceWidget* spaceWidget;
   QList<CustomPushButton*> enabledBtns;
};


#endif // TOPLEFTMENU_H
