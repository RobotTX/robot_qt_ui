#ifndef DISPLAYSELECTEDPOINT_H
#define DISPLAYSELECTEDPOINT_H

class Point;
class QMainWindow;
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;

#include <QLabel>
#include <memory>
#include <QWidget>


class DisplaySelectedPoint: public QWidget
{
public:
    DisplaySelectedPoint(QMainWindow* _parent);
    ~DisplaySelectedPoint();

    QPushButton* getBackButton(void) const { return backButton; }
    QPushButton* getMinusButton(void) const { return minusButton; }
    QPushButton* getMapButton(void) const { return mapButton; }
    QLabel* getNameLabel(void) const { return nameLabel; }
    QString getPointName(void) const { return nameLabel->text().right(nameLabel->text().length()-7); }

    void displayPointInfo(const std::shared_ptr<Point> _point);

private:
    QLabel* nameLabel;
    QLabel* posXLabel;
    QLabel* posYLabel;
    QVBoxLayout* layout;
    QPushButton* backButton;
    QPushButton* plusButton;
    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* eyeButton;
    QPushButton* editButton;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;
};

#endif // DISPLAYSELECTEDPOINT_H
