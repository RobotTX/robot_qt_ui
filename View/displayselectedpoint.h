#ifndef DISPLAYSELECTEDPOINT_H
#define DISPLAYSELECTEDPOINT_H

class Point;
class QMainWindow;
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class Point;
class QEvent;
class QKeyEvent;
class QLabel;

#include "Model/points.h"
#include <QLineEdit>
#include <memory>
#include <QWidget>
#include <QObject>

class DisplaySelectedPoint: public QWidget
{
        Q_OBJECT
public:
    DisplaySelectedPoint(QMainWindow* _parent, Points const& _points, const std::shared_ptr<Point> &_point = 0);
    ~DisplaySelectedPoint();

    QPushButton* getBackButton(void) const { return backButton; }
    QPushButton* getMinusButton(void) const { return minusButton; }
    QPushButton* getMapButton(void) const { return mapButton; }
    QPushButton* getEditButton(void) const { return editButton; }
    //QLabel* getNameLabel(void) const { return nameLabel; }
    QString getPointName(void) const { return nameEdit->text(); }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    std::shared_ptr<Point> getPoint(void) const { return point; }
    void setPoint(std::shared_ptr<Point> const& _point) { point = _point; }


public:
    void displayPointInfo(void);
    void mousePressEvent(QEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void displayPointInfo(const std::shared_ptr<Point> _point);


private slots:
    void updateName(QString newName);

signals:
    /// to notify the mapview that one of its points have been updated (in order to update the name that's displayed when the mouse is hovering over a point)
    void nameChanged(QString, QString);


private:
    QLineEdit* nameEdit;
    QHBoxLayout* nameLayout;
    //QLabel* nameLabel;
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
    std::shared_ptr<Point> point;
    QMainWindow* parent;
    Points points;
};

#endif // DISPLAYSELECTEDPOINT_H
