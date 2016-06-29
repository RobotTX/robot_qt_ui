#include "doubleclickablebutton.h"
#include <QDebug>

DoubleClickableButton::DoubleClickableButton(const int _id, const QString string, QWidget* parent): QPushButton(string, parent), id(_id)
{
}

void DoubleClickableButton::mouseDoubleClickEvent(QMouseEvent *event){
<<<<<<< HEAD
    qDebug() << "double click on button " << this;
=======
>>>>>>> d1be0061d32228edd44d673405cd8444a119db3d
    emit doubleClick(id);
}
