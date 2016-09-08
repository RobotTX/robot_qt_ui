/****************************************************************************
** Meta object code from reading C++ file 'updaterobotsthread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Controller/updaterobotsthread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'updaterobotsthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_UpdateRobotsThread_t {
    QByteArrayData data[14];
    char stringdata0[168];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_UpdateRobotsThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_UpdateRobotsThread_t qt_meta_stringdata_UpdateRobotsThread = {
    {
QT_MOC_LITERAL(0, 0, 18), // "UpdateRobotsThread"
QT_MOC_LITERAL(1, 19, 12), // "robotIsAlive"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 8), // "hostname"
QT_MOC_LITERAL(4, 42, 2), // "ip"
QT_MOC_LITERAL(5, 45, 5), // "mapId"
QT_MOC_LITERAL(6, 51, 4), // "ssid"
QT_MOC_LITERAL(7, 56, 5), // "stage"
QT_MOC_LITERAL(8, 62, 17), // "newConnectionSlot"
QT_MOC_LITERAL(9, 80, 16), // "disconnectedSlot"
QT_MOC_LITERAL(10, 97, 15), // "readTcpDataSlot"
QT_MOC_LITERAL(11, 113, 19), // "errorConnectionSlot"
QT_MOC_LITERAL(12, 133, 28), // "QAbstractSocket::SocketError"
QT_MOC_LITERAL(13, 162, 5) // "error"

    },
    "UpdateRobotsThread\0robotIsAlive\0\0"
    "hostname\0ip\0mapId\0ssid\0stage\0"
    "newConnectionSlot\0disconnectedSlot\0"
    "readTcpDataSlot\0errorConnectionSlot\0"
    "QAbstractSocket::SocketError\0error"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_UpdateRobotsThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    5,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    0,   50,    2, 0x08 /* Private */,
       9,    0,   51,    2, 0x08 /* Private */,
      10,    0,   52,    2, 0x08 /* Private */,
      11,    1,   53,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::QString, QMetaType::QString, QMetaType::Int,    3,    4,    5,    6,    7,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,

       0        // eod
};

void UpdateRobotsThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        UpdateRobotsThread *_t = static_cast<UpdateRobotsThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->robotIsAlive((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        case 1: _t->newConnectionSlot(); break;
        case 2: _t->disconnectedSlot(); break;
        case 3: _t->readTcpDataSlot(); break;
        case 4: _t->errorConnectionSlot((*reinterpret_cast< QAbstractSocket::SocketError(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 4:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAbstractSocket::SocketError >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (UpdateRobotsThread::*_t)(QString , QString , QString , QString , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&UpdateRobotsThread::robotIsAlive)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject UpdateRobotsThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_UpdateRobotsThread.data,
      qt_meta_data_UpdateRobotsThread,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *UpdateRobotsThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *UpdateRobotsThread::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_UpdateRobotsThread.stringdata0))
        return static_cast<void*>(const_cast< UpdateRobotsThread*>(this));
    return QThread::qt_metacast(_clname);
}

int UpdateRobotsThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void UpdateRobotsThread::robotIsAlive(QString _t1, QString _t2, QString _t3, QString _t4, int _t5)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
