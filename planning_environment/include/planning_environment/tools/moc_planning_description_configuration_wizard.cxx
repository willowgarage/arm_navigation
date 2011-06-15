/****************************************************************************
** Meta object code from reading C++ file 'planning_description_configuration_wizard.h'
**
** Created: Wed Jun 15 09:38:47 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "planning_description_configuration_wizard.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'planning_description_configuration_wizard.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PlanningDescriptionConfigurationWizard[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      29,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      63,   57,   40,   39, 0x0a,
     107,   94,   39,   39, 0x0a,
     138,   57,   39,   39, 0x2a,
     165,   39,   39,   39, 0x0a,
     187,   39,   39,   39, 0x0a,
     207,   39,   39,   39, 0x0a,
     234,   39,   39,   39, 0x0a,
     252,   39,   39,   39, 0x0a,
     279,   39,   39,   39, 0x0a,
     308,   39,   39,   39, 0x0a,
     335,   39,   39,   39, 0x0a,
     356,   39,   39,   39, 0x0a,
     377,   39,   39,   39, 0x0a,
     397,   39,   39,   39, 0x0a,
     416,   39,   39,   39, 0x0a,
     434,   39,   39,   39, 0x0a,
     454,   39,   39,   39, 0x0a,
     476,   39,   39,   39, 0x0a,
     496,   39,   39,   39, 0x0a,
     523,   39,   39,   39, 0x0a,
     550,   39,   39,   39, 0x0a,
     579,   39,   39,   39, 0x0a,
     610,   39,   39,   39, 0x0a,
     646,   39,   39,   39, 0x0a,
     685,   39,   39,   39, 0x0a,
     717,   39,   39,   39, 0x0a,
     750,   39,   39,   39, 0x0a,
     789,  784,   39,   39, 0x0a,
     811,   39,   39,   39, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PlanningDescriptionConfigurationWizard[] = {
    "PlanningDescriptionConfigurationWizard\0"
    "\0std::vector<int>\0table\0"
    "getSelectedRows(QTableWidget*)\0"
    "table,column\0toggleTable(QTableWidget*,int)\0"
    "toggleTable(QTableWidget*)\0"
    "defaultTogglePushed()\0oftenTogglePushed()\0"
    "occasionallyTogglePushed()\0dofTogglePushed()\0"
    "selectJointButtonClicked()\0"
    "deselectJointButtonClicked()\0"
    "deleteGroupButtonClicked()\0"
    "acceptChainClicked()\0acceptGroupClicked()\0"
    "baseLinkTreeClick()\0tipLinkTreeClick()\0"
    "validateDoneBox()\0groupTableClicked()\0"
    "defaultTableClicked()\0oftenTableClicked()\0"
    "occasionallyTableClicked()\0"
    "dofSelectionTableChanged()\0"
    "oftenCollisionTableChanged()\0"
    "defaultCollisionTableChanged()\0"
    "occasionallyCollisionTableChanged()\0"
    "generateOccasionallyInCollisionTable()\0"
    "generateOftenInCollisionTable()\0"
    "generateAlwaysInCollisionTable()\0"
    "generateDefaultInCollisionTable()\0"
    "file\0fileSelected(QString)\0writeFiles()\0"
};

const QMetaObject PlanningDescriptionConfigurationWizard::staticMetaObject = {
    { &QWizard::staticMetaObject, qt_meta_stringdata_PlanningDescriptionConfigurationWizard,
      qt_meta_data_PlanningDescriptionConfigurationWizard, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PlanningDescriptionConfigurationWizard::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PlanningDescriptionConfigurationWizard::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PlanningDescriptionConfigurationWizard::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlanningDescriptionConfigurationWizard))
        return static_cast<void*>(const_cast< PlanningDescriptionConfigurationWizard*>(this));
    return QWizard::qt_metacast(_clname);
}

int PlanningDescriptionConfigurationWizard::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizard::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: { std::vector<int> _r = getSelectedRows((*reinterpret_cast< QTableWidget*(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<int>*>(_a[0]) = _r; }  break;
        case 1: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1]))); break;
        case 3: defaultTogglePushed(); break;
        case 4: oftenTogglePushed(); break;
        case 5: occasionallyTogglePushed(); break;
        case 6: dofTogglePushed(); break;
        case 7: selectJointButtonClicked(); break;
        case 8: deselectJointButtonClicked(); break;
        case 9: deleteGroupButtonClicked(); break;
        case 10: acceptChainClicked(); break;
        case 11: acceptGroupClicked(); break;
        case 12: baseLinkTreeClick(); break;
        case 13: tipLinkTreeClick(); break;
        case 14: validateDoneBox(); break;
        case 15: groupTableClicked(); break;
        case 16: defaultTableClicked(); break;
        case 17: oftenTableClicked(); break;
        case 18: occasionallyTableClicked(); break;
        case 19: dofSelectionTableChanged(); break;
        case 20: oftenCollisionTableChanged(); break;
        case 21: defaultCollisionTableChanged(); break;
        case 22: occasionallyCollisionTableChanged(); break;
        case 23: generateOccasionallyInCollisionTable(); break;
        case 24: generateOftenInCollisionTable(); break;
        case 25: generateAlwaysInCollisionTable(); break;
        case 26: generateDefaultInCollisionTable(); break;
        case 27: fileSelected((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 28: writeFiles(); break;
        default: ;
        }
        _id -= 29;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
