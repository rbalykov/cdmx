/*
 * util.h
 */

#ifndef INCLUDE_UTIL_H_
#define INCLUDE_UTIL_H_

#include <uapi/linux/stat.h>

/*******************************************************************************
 ******************************************************************************/

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define TO_RANGE(a,min,max) (MIN(max,MAX(min,a)))

#define CHMOD_RW	(S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
#define CHMOD_RO	(S_IRUSR | S_IRGRP | S_IROTH)

/*******************************************************************************
 ******************************************************************************/

#endif /* INCLUDE_UTIL_H_ */
