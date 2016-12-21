#include "mergedialog.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include "mergemapwidget.h"
#include "View/custompushbutton.h"
#include "assert.h"

MergeDialog::MergeDialog(QWidget* parent): nbMapsChecked(0) {

    setWindowTitle("Merge two maps automatically");
    mainLayout = new QVBoxLayout(this);

    /// the layout for the cancel and merge buttons
    buttonsLayout = new QHBoxLayout();

    /// the layout containing the buttons used to select the different maps
    layout = new QGridLayout();

    QLabel* label = new QLabel("Select exactly two maps and click \"merge\" to merge them", this);
    mainLayout->addWidget(label);

    for(int i = 0; i < static_cast<MergeMapWidget*>(parent)->getListWidget()->count(); i++){
        MergeMapListItemWidget* item = static_cast<MergeMapListItemWidget*>(static_cast<MergeMapWidget*>(parent)->getListWidget()->itemWidget(static_cast<MergeMapWidget*>(parent)->getListWidget()->item(i)));
        qDebug() << "MergeDialog" << item->getFileNameLabel();
        CustomPushButton* button = new CustomPushButton(item->getFileNameLabel(), this);
        connect(button, SIGNAL(clicked(bool)), this, SLOT(updateNbMapsChecked(bool)));
        /// those buttons can be selected to perform a merge operation on the corresponding maps
        button->setCheckable(true);
        /// we group the buttons by columns of 2 like in elementary school
        layout->addWidget(button, i/2, i%2);
    }

    CustomPushButton* cancelButton = new CustomPushButton("Cancel", this);
    mergeButton = new CustomPushButton("Merge", this);
    mergeButton->setToolTip("Select exactly two maps and click to merge them");
    mergeButton->setEnabled(false);
    buttonsLayout->addWidget(cancelButton);
    buttonsLayout->addWidget(mergeButton);

    mainLayout->addLayout(layout);
    mainLayout->addLayout(buttonsLayout);
    setGeometry(500, 500, 150, 150);

    connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()));
    connect(mergeButton, SIGNAL(clicked()), this, SLOT(emitMapsToMerge()));
}

void MergeDialog::updateNbMapsChecked(bool checked){
    (checked) ? nbMapsChecked++ : nbMapsChecked-- ;
    /// if two maps exactly are chosen then we allow the merge
    mergeButton->setEnabled((nbMapsChecked == 2) ? true : false);
}

void MergeDialog::emitMapsToMerge(){
    unsigned int first_map_id(-1);
    unsigned int second_map_id(-1);
    bool found_first(false);
    for(int idx = 0; idx < layout->count(); idx++){
        QWidget * const item = layout->itemAt(idx)->widget();
        if(item && static_cast<QAbstractButton *>(item)->isChecked()){
            if(!found_first){
                first_map_id = idx;
                found_first = true;
            } else
                second_map_id = idx;
        }
    }
    assert(first_map_id != -1u && second_map_id != -1u);
    emit mapsToMerge(first_map_id, second_map_id);
}
