#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(RTCONTROL_LIB)
#  define RTCONTROL_EXPORT Q_DECL_EXPORT
# else
#  define RTCONTROL_EXPORT Q_DECL_IMPORT
# endif
#else
# define RTCONTROL_EXPORT
#endif
