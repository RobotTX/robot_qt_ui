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

    /// **                    UPDATES THE MODEL WITH THE SETTINGS CONTAINED IN THE FILES ** ///
    QString helpFile = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + "help.txt";
    std::ifstream fileHelp(helpFile.toStdString(), std::ios::in);
    if(fileHelp) {
        int helpNeeded;
        fileHelp >> helpNeeded;
        settings->setHelpNeeded(helpNeeded);
        fileHelp.close();
    } else
        settings->setHelpNeeded(true);

    QString fileStr = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + "mapChoice.txt";
    std::ifstream file(fileStr.toStdString(), std::ios::in);
    if(file) {
        int settingMapChoice;
        file >> settingMapChoice;
        settings->setMapChoice(settingMapChoice);
        file.close();
    } else
        settings->setMapChoice(0);

    QString file_battery = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + "battery_threshold.txt";
    std::ifstream battery_file(file_battery.toStdString(), std::ios::in);
    if(battery_file) {
        int battery_thresh;
        battery_file >> battery_thresh;
        settings->setBatteryThreshold(battery_thresh);
        battery_file.close();
    } else
        settings->setBatteryThreshold(20);

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
    QString helpFile = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + "help.txt";
    std::ofstream fileHelp(helpFile.toStdString(), std::ios::out | std::ios::trunc);

    if(fileHelp) {
        fileHelp << "1";
        fileHelp.close();
    }
    /// we update no matter what because we always want the model and the view to be
    /// consistent with each other
    settings->resetSettings();
}

void SettingsController::updateBatteryThreshold(const int value){
    qDebug() << "SettingsController::updateBatteryThreshold called" << value;
    QString file_battery = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + "battery_threshold.txt";
    std::ofstream battery_file(file_battery.toStdString(), std::ios::out | std::ios::trunc);

    if(battery_file) {
        battery_file << value;
        battery_file.close();
    }
    /// we update no matter what because we always want the model and the view to be
    /// consistent with each other
    settings->setBatteryThreshold(value);
}

void SettingsController::setMapChoice(const int choice){
    qDebug() << "SettingsController::setMapChoice called" << choice;
    QString fileStr = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + "mapChoice.txt";
    std::ofstream file(fileStr.toStdString(), std::ios::out | std::ios::trunc);

    if(file) {
        file << choice;
        file.close();
    }
    /// we update no matter what because we always want the model and the view to be
    /// consistent with each other
    settings->setMapChoice(choice);
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
    settings->removeRobot(name);
    QMapIterator<int, QPair<QString, bool> > it(settings->getIDtoNameMap());
    while(it.hasNext()){
        it.next();
        if(!it.value().first.compare(name))
            view->removeRobot(it.key());
    }
}

/// called when a user does not want to see the help messages
void SettingsController::hideTutorial(){
    settings->setHelpNeeded(false);
    QString helpFile = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + "help.txt";
    std::ofstream fileHelp(helpFile.toStdString(), std::ios::out | std::ios::trunc);
    if(fileHelp) {
        fileHelp << "0";
        fileHelp.close();
    }
}
