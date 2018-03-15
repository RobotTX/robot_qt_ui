#include "speech.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Speech::Speech(const QString _name, const QString _tts, QObject *parent) :
    QObject(parent), name(_name), tts(_tts)
{}
