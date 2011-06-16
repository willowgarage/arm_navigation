/****************************************************************************
** Meta object code from reading C++ file 'planning_description_configuration_wizard.h'
**
** Created: Wed Jun 15 17:08:31 2011
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
      39,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      49,   40,   39,   39, 0x05,

 // slots: signature, parameters, type, tag, flags
      92,   86,   69,   39, 0x0a,
     134,  123,   39,   39, 0x0a,
     158,  123,   39,   39, 0x0a,
     182,  123,   39,   39, 0x0a,
     210,  123,   39,   39, 0x0a,
     234,  123,   39,   39, 0x0a,
     260,  123,   39,   39, 0x0a,
     284,  123,   39,   39, 0x0a,
     325,  312,   39,   39, 0x0a,
     356,   86,   39,   39, 0x2a,
     383,   39,   39,   39, 0x0a,
     405,   39,   39,   39, 0x0a,
     425,   39,   39,   39, 0x0a,
     452,   39,   39,   39, 0x0a,
     470,   39,   39,   39, 0x0a,
     497,   39,   39,   39, 0x0a,
     526,   39,   39,   39, 0x0a,
     553,   39,   39,   39, 0x0a,
     574,   39,   39,   39, 0x0a,
     595,   39,   39,   39, 0x0a,
     615,   39,   39,   39, 0x0a,
     634,   39,   39,   39, 0x0a,
     652,   39,   39,   39, 0x0a,
     672,   39,   39,   39, 0x0a,
     694,   39,   39,   39, 0x0a,
     714,   39,   39,   39, 0x0a,
     741,   39,   39,   39, 0x0a,
     768,   39,   39,   39, 0x0a,
     797,   39,   39,   39, 0x0a,
     828,   39,   39,   39, 0x0a,
     864,   39,   39,   39, 0x0a,
     903,   39,   39,   39, 0x0a,
     935,   39,   39,   39, 0x0a,
     968,   39,   39,   39, 0x0a,
    1007, 1002,   39,   39, 0x0a,
    1029,   39,   39,   39, 0x0a,
    1042,   39,   39,   39, 0x0a,
    1058,   39,   39,   39, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PlanningDescriptionConfigurationWizard[] = {
    "PlanningDescriptionConfigurationWizard\0"
    "\0progress\0changeProgress(int)\0"
    "std::vector<int>\0table\0"
    "getSelectedRows(QTableWidget*)\0"
    "checkState\0easyButtonToggled(bool)\0"
    "hardButtonToggled(bool)\0"
    "verySafeButtonToggled(bool)\0"
    "safeButtonToggled(bool)\0"
    "normalButtonToggled(bool)\0"
    "fastButtonToggled(bool)\0"
    "veryFastButtonToggled(bool)\0table,column\0"
    "toggleTable(QTableWidget*,int)\0"
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
    "autoConfigure()\0update()\0"
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
        case 0: changeProgress((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: { std::vector<int> _r = getSelectedRows((*reinterpret_cast< QTableWidget*(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<int>*>(_a[0]) = _r; }  break;
        case 2: easyButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: hardButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: verySafeButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: safeButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: normalButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: fastButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: veryFastButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 10: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1]))); break;
        case 11: defaultTogglePushed(); break;
        case 12: oftenTogglePushed(); break;
        case 13: occasionallyTogglePushed(); break;
        case 14: dofTogglePushed(); break;
        case 15: selectJointButtonClicked(); break;
        case 16: deselectJointButtonClicked(); break;
        case 17: deleteGroupButtonClicked(); break;
        case 18: acceptChainClicked(); break;
        case 19: acceptGroupClicked(); break;
        case 20: baseLinkTreeClick(); break;
        case 21: tipLinkTreeClick(); break;
        case 22: validateDoneBox(); break;
        case 23: groupTableClicked(); break;
        case 24: defaultTableClicked(); break;
        case 25: oftenTableClicked(); break;
        case 26: occasionallyTableClicked(); break;
        case 27: dofSelectionTableChanged(); break;
        case 28: oftenCollisionTableChanged(); break;
        case 29: defaultCollisionTableChanged(); break;
        case 30: occasionallyCollisionTableChanged(); break;
        case 31: generateOccasionallyInCollisionTable(); break;
        case 32: generateOftenInCollisionTable(); break;
        case 33: generateAlwaysInCollisionTable(); break;
        case 34: generateDefaultInCollisionTable(); break;
        case 35: fileSelected((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 36: writeFiles(); break;
        case 37: autoConfigure(); break;
        case 38: update(); break;
        default: ;
        }
        _id -= 39;
    }
    return _id;
}

// SIGNAL 0
void PlanningDescriptionConfigurationWizard::changeProgress(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
