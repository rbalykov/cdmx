/*
 * debug.h
 */

#ifndef INCLUDE_DEBUG_H_
#define INCLUDE_DEBUG_H_

#include <linux/module.h>
#include <linux/printk.h>

/*******************************************************************************
 * DEBUG MACROS
 ******************************************************************************/

//TODO: refactor to use pr_debug()

#define _pp(severity, format, args...) \
  printk(severity "%s: %d: %s: " format "\n", THIS_MODULE->name, __LINE__, __func__, ##args)

#define K_WARN(args...) 	_pp(KERN_WARNING,	args)
#define K_NOTE(args...) 	_pp(KERN_NOTICE, 	args)
#define K_INFO(args...) 	_pp(KERN_INFO, 		args)
#define K_ERR(args...) 		_pp(KERN_ERR, 		args)
#define K_DEBUG(args...) 	_pp(KERN_DEBUG, 	args)


/*******************************************************************************
 ******************************************************************************/

#endif /* INCLUDE_DEBUG_H_ */
