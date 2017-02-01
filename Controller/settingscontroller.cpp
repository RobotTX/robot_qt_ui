#include "settingscontroller.h"
#include "View/settingswidget.h"
#include <QComboBox>
#include <QButtonGroup>
#include <QDir>
#include <fstream>
#include <Controller/mainwindow.h>
#include "View/custompushbutton.h"

SettingsController::SettingsController(QWidget* parent): QObject(parent)
{
    /// Model, contains the actual information
    settings = QPointer<Settings> (new Settings());

    /// Get the settings from the file
    deserializeSettings();

    /// Creates the graphical object where a user can change the settings to his preferences
    createSettingsView();

    /// relays the change of status of a laser to the mainwindow which needs to use the command controller
    connect(this, SIGNAL(activateLaser(QString, bool)), static_cast<MainWindow*> (this->parent()), SLOT(activateLaserSlot(QString, bool)));
}

SettingsController::~SettingsController(){
    delete view;
}

void SettingsController::createSettingsView(void){

    qDebug() << "SettingsController::createSettingsView called";

    view = new SettingsWidget(*settings);

    /// set up the connects to propagate the changes in the view to the model ///
    connect(view->getBatterySlider(), SIGNAL(valueChanged(int)), this, SLOT(updateBatteryThreshold(int)));

    /// to turn on the laser feedback or not ( to display obstacles in real time )
    connect(view->getRobotLaserButtonGroup(), SIGNAL(buttonToggled(int, bool)), this, SLOT(updateLaserStatus(int, bool)));

    connect(view, SIGNAL(updateMapChoice(int)), this, SLOT(setMapChoice(int)));

    connect(view->getHelpButton(), SIGNAL(clicked()), this, SLOT(setHelpNeeded()));
}

void SettingsController::hideView(){
    view->hide();
}

void SettingsController::showView(){
    createSettingsView();
    view->show();
}

void SettingsController::setHelpNeeded(){
    qDebug() << "SettingsController::setHelpNeeded";
    /// we update no matter what because we always want the model and the view to be
    /// consistent with each other
    settings->resetSettings();
    serializeSettings();
}

void SettingsController::updateBatteryThreshold(const int value){
    qDebug() << "SettingsController::updateBatteryThreshold called" << value;
    /// we update no matter what because we always want the model and the view to be
    /// consistent with each other
    settings->setBatteryThreshold(value);
    serializeSettings();
}

void SettingsController::setMapChoice(const int choice){
    qDebug() << "SettingsController::setMapChoice called" << choice;
    /// we update no matter what because we always want the model and the view to be
    /// consistent with each other
    settings->setMapChoice(choice);
    serializeSettings();
}

void SettingsController::updateLaserStatus(const int id, const bool status){
    qDebug() << "SettingsController::updateLaserStatus called" << id << status;
    /// if we can update the file we also update the model
    if(settings->setLaserStatus(id, status)) {
        QMapIterator<int, QPair<QString, bool> > it(settings->getIDtoNameMap());
        while(it.hasNext()){
            it.next();
            if(it.key() == id)
                emit activateLaser(it.value().first, status);
        }
    }
}

void SettingsController::addRobot(const QString name){
    /// enables the laser feedback for a particular robot
    settings->addRobot(name);
    QMapIterator<int, QPair<QString, bool> > it(settings->getIDtoNameMap());
    /// this seems a little repetitive but it ensures that the model and the view are consistent with each other
    while(it.hasNext()){
        it.next();
        if(!it.value().first.compare(name)){
            view->addRobot(name, settings->getCurrentId(), it.value().second);
            break;
        }
    }
}

void SettingsController::removeRobot(const QString name){
    /// disables the laser feedback for a particular robot
    settings->removeRobot(name);
    QMapIterator<int, QPair<QString, bool> > it(settings->getIDtoNameMap());
    while(it.hasNext()){
        it.next();
        if(!it.value().first.compare(name))
            view->removeRobot(it.key());
    }
}

/// called when a user does not want to see the help messages
void SettingsController::hideTutorial(const bool messageNeeded, const QString feature){
    settings->setHelpNeeded(messageNeeded, feature);
    serializeSettings();
}

void SettingsController::serializeSettings() const {
    QFile settingsFile(QDir::currentPath() + QDir::separator() + "settings.dat");
    settingsFile.resize(0);
    settingsFile.open(QIODevice::ReadWrite);
    QDataStream out(&settingsFile);
    out << *settings;
    settingsFile.close();
}

void SettingsController::deserializeSettings(){
    QFile fileRead(QDir::currentPath() + QDir::separator() + "settings.dat");
    if(fileRead.open(QIODevice::ReadOnly)){
        /// read the data serialized from the file
        QDataStream in(&fileRead);
        in >> *settings;
        fileRead.close();
    }
}
