/*
 * debug.h
 */

#ifndef INCLUDE_DEBUG_H_
#define INCLUDE_DEBUG_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/export.h>

/*******************************************************************************
 * DEBUG MACROS
 ******************************************************************************/

//#if defined (DYNAMIC_DEBUG_MODULE)
#if defined (USE_SIMPLE_DYNDBG)

#define K_WARN(args...) 	pr_warn(args)
#define K_NOTE(args...) 	pr_note(args)
#define K_INFO(args...) 	pr_info(args)
#define K_ERR(args...) 		pr_err(args)
#define K_DEBUG(args...) 	pr_debug(args)

#else

#define K_WARN(fmt, args...) 	pr_warn("%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_NOTE(fmt, args...) 	pr_note("%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_INFO(fmt, args...) 	pr_info("%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_ERR(fmt, args...) 	pr_err("%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_DEBUG(fmt, args...) 	pr_debug("%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)

#endif // USE_SIMPLE_DYNDBG

//#else
//
//#define _pp(severity, format, args...) \
//  printk(severity "%s: %d: %s: " format "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
//
//#define K_WARN(args...) 	_pp(KERN_WARNING,	args)
//#define K_NOTE(args...) 	_pp(KERN_NOTICE, 	args)
//#define K_INFO(args...) 	_pp(KERN_INFO, 		args)
//#define K_ERR(args...) 		_pp(KERN_ERR, 		args)
//#define K_DEBUG(args...) 	_pp(KERN_DEBUG, 	args)
//
//
//#endif // DYNAMIC_DEBUG_MODULE
/*******************************************************************************
 ******************************************************************************/

#endif /* INCLUDE_DEBUG_H_ */
