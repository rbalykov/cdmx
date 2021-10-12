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

#if defined(CONFIG_DYNAMIC_DEBUG) || \
	(defined(CONFIG_DYNAMIC_DEBUG_CORE) && defined(DYNAMIC_DEBUG_MODULE))

#define K_WARN(fmt, args...) 	pr_warn(fmt "\n", ##args)
#define K_NOTE(fmt, args...) 	pr_note(fmt "\n", ##args)
#define K_INFO(fmt, args...) 	pr_info(fmt "\n", ##args)
#define K_ERR(fmt, args...) 	pr_err(fmt "\n", ##args)
#define K_DEBUG(fmt, args...) 	pr_debug(fmt "\n", ##args)

#else

#define K_WARN(fmt, args...) 	printk(KERN_WARNING "%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_NOTE(fmt, args...) 	printk(KERN_NOTICE "%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_INFO(fmt, args...) 	printk(KERN_INFO "%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_ERR(fmt, args...) 	printk(KERN_ERR "%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)
#define K_DEBUG(fmt, args...) 	printk(KERN_DEBUG "%s: %d: %s: " fmt "\n", THIS_MODULE->name, __LINE__, __func__, ##args)

#endif // USE_SIMPLE_DYNDBG

/*******************************************************************************
 ******************************************************************************/

#endif /* INCLUDE_DEBUG_H_ */
