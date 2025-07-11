#ifndef COCO_HARDWARE__VISIBILITY_CONTROL_H_
#define COCO_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COCO_HARDWARE_EXPORT __attribute__((dllexport))
#define COCO_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define COCO_HARDWARE_EXPORT __declspec(dllexport)
#define COCO_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef COCO_HARDWARE_BUILDING_DLL
#define COCO_HARDWARE_PUBLIC COCO_HARDWARE_EXPORT
#else
#define COCO_HARDWARE_PUBLIC COCO_HARDWARE_IMPORT
#endif
#define COCO_HARDWARE_PUBLIC_TYPE COCO_HARDWARE_PUBLIC
#define COCO_HARDWARE_LOCAL
#else
#define COCO_HARDWARE_EXPORT __attribute__((visibility("default")))
#define COCO_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define COCO_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define COCO_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define COCO_HARDWARE_PUBLIC
#define COCO_HARDWARE_LOCAL
#endif
#define COCO_HARDWARE_PUBLIC_TYPE
#endif

#endif  // COCO_HARDWARE__VISIBILITY_CONTROL_H_
