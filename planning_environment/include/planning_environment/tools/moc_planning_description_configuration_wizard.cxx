/****************************************************************************
** Meta object code from reading C++ file 'planning_description_configuration_wizard.h'
**
** Created: Fri Jun 10 11:37:12 2011
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
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      40,   39,   39,   39, 0x0a,
      67,   39,   39,   39, 0x0a,
      96,   39,   39,   39, 0x0a,
     123,   39,   39,   39, 0x0a,
     144,   39,   39,   39, 0x0a,
     165,   39,   39,   39, 0x0a,
     185,   39,   39,   39, 0x0a,
     204,   39,   39,   39, 0x0a,
     231,   39,   39,   39, 0x0a,
     260,   39,   39,   39, 0x0a,
     296,   39,   39,   39, 0x0a,
     335,   39,   39,   39, 0x0a,
     367,   39,   39,   39, 0x0a,
     400,   39,   39,   39, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PlanningDescriptionConfigurationWizard[] = {
    "PlanningDescriptionConfigurationWizard\0"
    "\0selectJointButtonClicked()\0"
    "deselectJointButtonClicked()\0"
    "deleteGroupButtonClicked()\0"
    "acceptChainClicked()\0acceptGroupClicked()\0"
    "baseLinkTreeClick()\0tipLinkTreeClick()\0"
    "dofSelectionTableChanged()\0"
    "oftenCollisionTableChanged()\0"
    "occasionallyCollisionTableChanged()\0"
    "generateOccasionallyInCollisionTable()\0"
    "generateOftenInCollisionTable()\0"
    "generateAlwaysInCollisionTable()\0"
    "writeFiles()\0"
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
        case 0: selectJointButtonClicked(); break;
        case 1: deselectJointButtonClicked(); break;
        case 2: deleteGroupButtonClicked(); break;
        case 3: acceptChainClicked(); break;
        case 4: acceptGroupClicked(); break;
        case 5: baseLinkTreeClick(); break;
        case 6: tipLinkTreeClick(); break;
        case 7: dofSelectionTableChanged(); break;
        case 8: oftenCollisionTableChanged(); break;
        case 9: occasionallyCollisionTableChanged(); break;
        case 10: generateOccasionallyInCollisionTable(); break;
        case 11: generateOftenInCollisionTable(); break;
        case 12: generateAlwaysInCollisionTable(); break;
        case 13: writeFiles(); break;
        default: ;
        }
        _id -= 14;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
