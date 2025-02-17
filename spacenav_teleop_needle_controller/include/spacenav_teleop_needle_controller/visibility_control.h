#ifndef SPACENAV_TELEOP_NEEDLE_CONTROLLER__VISIBILITY_CONTROL_H_
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_EXPORT __attribute__((dllexport))
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_EXPORT __declspec(dllexport)
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef SPACENAV_TELEOP_NEEDLE_CONTROLLER_BUILDING_DLL
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC SPACENAV_TELEOP_NEEDLE_CONTROLLER_EXPORT
#else
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC SPACENAV_TELEOP_NEEDLE_CONTROLLER_IMPORT
#endif
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC_TYPE SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_LOCAL
#else
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_LOCAL
#endif
#define SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC_TYPE
#endif

#endif // SPACENAV_TELEOP_NEEDLE_CONTROLLER__VISIBILITY_CONTROL_H_
