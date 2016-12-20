#include "mergedialog.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include "mergemapwidget.h"
#include "View/custompushbutton.h"

MergeDialog::MergeDialog(QWidget* parent): nbMapsChecked(0) {

    setWindowTitle("Merge two maps automatically");
    mainLayout = new QVBoxLayout(this);
    buttonsLayout = new QHBoxLayout();
    layout = new QGridLayout();

    QLabel* label = new QLabel("Select exactly two maps and click \"merge\" to merge them", this);
    mainLayout->addWidget(label);

    for(int i = 0; i < static_cast<MergeMapWidget*>(parent)->getListWidget()->count(); i++){
        MergeMapListItemWidget* item = static_cast<MergeMapListItemWidget*>(static_cast<MergeMapWidget*>(parent)->getListWidget()->itemWidget(static_cast<MergeMapWidget*>(parent)->getListWidget()->item(i)));
        qDebug() << "MergeDialog" << item->getFileNameLabel();
        CustomPushButton* button = new CustomPushButton(item->getFileNameLabel(), this);
        connect(button, SIGNAL(clicked(bool)), this, SLOT(updateNbMapsChecked(bool)));
        button->setCheckable(true);
        layout->addWidget(button, i/2, i%2);
    }
    CustomPushButton* cancelButton = new CustomPushButton("Cancel", this);
    mergeButton = new CustomPushButton("Merge", this);
    mergeButton->setEnabled(false);
    buttonsLayout->addWidget(cancelButton);
    buttonsLayout->addWidget(mergeButton);


    mainLayout->addLayout(layout);
    mainLayout->addLayout(buttonsLayout);
    setGeometry(500, 500, 150, 150);

    connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()));
    connect(mergeButton, SIGNAL(clicked()), parent, SLOT(mergeAutomatically()));
}

void MergeDialog::updateNbMapsChecked(bool checked){
    (checked) ? nbMapsChecked++ : nbMapsChecked-- ;
    /// if two maps exactly are chosen then we allow the merge
    mergeButton->setEnabled((nbMapsChecked == 2) ? true : false);
}
