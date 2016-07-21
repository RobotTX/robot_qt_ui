#ifndef GROUPBUTTONGROUP_H
#define GROUPBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;
class QMouseEvent;
class CustomizedLineEdit;

#include <QButtonGroup>
#include <QWidget>
#include <memory>
#include <QObject>

class GroupButtonGroup: public QWidget
{
    Q_OBJECT
public:
    GroupButtonGroup(std::shared_ptr<Points> const& _points, QWidget *_parent);
    ~GroupButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    CustomizedLineEdit* getModifyEdit(void) const { return modifyEdit; }
    QVBoxLayout* getLayout(void) const { return layout; }
    int getIndexModifyEdit(void) const { return indexModifyEdit; }
    void setIndexModifyEdit(const int index) { indexModifyEdit = index; }
    QString formatName(const QString name) const;

public:
    void deleteButtons(void);
    void update(const Points& _points);
    void uncheck(void);
    void setEnabled(const bool enable);

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);

signals:
    void doubleClick(int);
    void updateConnectionsRequest();
    void modifyEditReconnection();
    void codeEditGroup(int);

public slots:
    int checkEditGroupName(QString name);

private:
    CustomizedLineEdit* modifyEdit;
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QWidget* parent;
    /// to avoid resizing of the icons after deletions of points and groups
    const QSize BUTTON_SIZE = parentWidget()->size()/2;
    int indexModifyEdit;
    std::shared_ptr<Points> points;
};

#endif // GROUPBUTTONGROUP_H
