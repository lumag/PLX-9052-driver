#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#include "plx9052-26.c"
#else
#include "plx9052-24.c"
#endif
