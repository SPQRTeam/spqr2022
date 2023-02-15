#ifndef G2O_CONFIG_H
#define G2O_CONFIG_H


/* #undef G2O_HAVE_OPENGL */
/* #undef G2O_OPENGL_FOUND */
/* #undef G2O_OPENMP */
#define G2O_SHARED_LIBS 1
#define G2O_LGPL_SHARED_LIBS 1

// available sparse matrix libraries
/* #undef G2O_HAVE_CHOLMOD */
#define G2O_HAVE_CSPARSE 1

#define G2O_CXX_COMPILER "GNU /home/opennao/jenkins/workspace/yocto-rootfs/build/tmp/sysroots/x86_64-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++"

#ifdef __cplusplus
#include <g2o/core/eigen_types.h>
#endif

#endif
