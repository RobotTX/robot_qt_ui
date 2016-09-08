/****************************************************************************
** Meta object code from reading C++ file 'cmdrobotthread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Controller/cmdrobotthread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cmdrobotthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_CmdRobotThread_t {
    QByteArrayData data[18];
    char stringdata0[226];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CmdRobotThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CmdRobotThread_t qt_meta_stringdata_CmdRobotThread = {
    {
QT_MOC_LITERAL(0, 0, 14), // "CmdRobotThread"
QT_MOC_LITERAL(1, 15, 11), // "robotIsDead"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 8), // "hostname"
QT_MOC_LITERAL(4, 37, 2), // "ip"
QT_MOC_LITERAL(5, 40, 13), // "connectedSlot"
QT_MOC_LITERAL(6, 54, 9), // "errorSlot"
QT_MOC_LITERAL(7, 64, 28), // "QAbstractSocket::SocketError"
QT_MOC_LITERAL(8, 93, 5), // "error"
QT_MOC_LITERAL(9, 99, 14), // "onStateChanged"
QT_MOC_LITERAL(10, 114, 28), // "QAbstractSocket::SocketState"
QT_MOC_LITERAL(11, 143, 16), // "disconnectedSlot"
QT_MOC_LITERAL(12, 160, 15), // "readTcpDataSlot"
QT_MOC_LITERAL(13, 176, 19), // "changeRobotNameSlot"
QT_MOC_LITERAL(14, 196, 4), // "name"
QT_MOC_LITERAL(15, 201, 11), // "sendCommand"
QT_MOC_LITERAL(16, 213, 3), // "cmd"
QT_MOC_LITERAL(17, 217, 8) // "pingSlot"

    },
    "CmdRobotThread\0robotIsDead\0\0hostname\0"
    "ip\0connectedSlot\0errorSlot\0"
    "QAbstractSocket::SocketError\0error\0"
    "onStateChanged\0QAbstractSocket::SocketState\0"
    "disconnectedSlot\0readTcpDataSlot\0"
    "changeRobotNameSlot\0name\0sendCommand\0"
    "cmd\0pingSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CmdRobotThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   59,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   64,    2, 0x08 /* Private */,
       6,    1,   65,    2, 0x08 /* Private */,
       9,    1,   68,    2, 0x08 /* Private */,
      11,    0,   71,    2, 0x08 /* Private */,
      12,    0,   72,    2, 0x08 /* Private */,
      13,    1,   73,    2, 0x08 /* Private */,
      15,    1,   76,    2, 0x08 /* Private */,
      17,    0,   79,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    3,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   14,
    QMetaType::Void, QMetaType::QString,   16,
    QMetaType::Void,

       0        // eod
};

void CmdRobotThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CmdRobotThread *_t = static_cast<CmdRobotThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->robotIsDead((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 1: _t->connectedSlot(); break;
        case 2: _t->errorSlot((*reinterpret_cast< QAbstractSocket::SocketError(*)>(_a[1]))); break;
        case 3: _t->onStateChanged((*reinterpret_cast< QAbstractSocket::SocketState(*)>(_a[1]))); break;
        case 4: _t->disconnectedSlot(); break;
        case 5: _t->readTcpDataSlot(); break;
        case 6: _t->changeRobotNameSlot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->sendCommand((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 8: _t->pingSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAbstractSocket::SocketError >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAbstractSocket::SocketState >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (CmdRobotThread::*_t)(QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CmdRobotThread::robotIsDead)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject CmdRobotThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_CmdRobotThread.data,
      qt_meta_data_CmdRobotThread,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CmdRobotThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CmdRobotThread::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CmdRobotThread.stringdata0))
        return static_cast<void*>(const_cast< CmdRobotThread*>(this));
    return QThread::qt_metacast(_clname);
}

int CmdRobotThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void CmdRobotThread::robotIsDead(QString _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
