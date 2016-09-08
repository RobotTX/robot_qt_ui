/****************************************************************************
** Meta object code from reading C++ file 'sendnewmapthread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Controller/sendnewmapthread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'sendnewmapthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_SendNewMapThread_t {
    QByteArrayData data[7];
    char stringdata0[94];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SendNewMapThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SendNewMapThread_t qt_meta_stringdata_SendNewMapThread = {
    {
QT_MOC_LITERAL(0, 0, 16), // "SendNewMapThread"
QT_MOC_LITERAL(1, 17, 23), // "doneSendingNewMapSignal"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 13), // "connectedSlot"
QT_MOC_LITERAL(4, 56, 16), // "disconnectedSlot"
QT_MOC_LITERAL(5, 73, 16), // "writeTcpDataSlot"
QT_MOC_LITERAL(6, 90, 3) // "cmd"

    },
    "SendNewMapThread\0doneSendingNewMapSignal\0"
    "\0connectedSlot\0disconnectedSlot\0"
    "writeTcpDataSlot\0cmd"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SendNewMapThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   35,    2, 0x08 /* Private */,
       4,    0,   36,    2, 0x08 /* Private */,
       5,    1,   37,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QByteArray,    6,

       0        // eod
};

void SendNewMapThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SendNewMapThread *_t = static_cast<SendNewMapThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->doneSendingNewMapSignal(); break;
        case 1: _t->connectedSlot(); break;
        case 2: _t->disconnectedSlot(); break;
        case 3: _t->writeTcpDataSlot((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (SendNewMapThread::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&SendNewMapThread::doneSendingNewMapSignal)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject SendNewMapThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_SendNewMapThread.data,
      qt_meta_data_SendNewMapThread,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SendNewMapThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SendNewMapThread::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SendNewMapThread.stringdata0))
        return static_cast<void*>(const_cast< SendNewMapThread*>(this));
    return QThread::qt_metacast(_clname);
}

int SendNewMapThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void SendNewMapThread::doneSendingNewMapSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
