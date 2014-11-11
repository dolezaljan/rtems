/**
 * @file
 *
 * @ingroup i386_pc386
 *
 * @brief Sizes of Global and Interrupt descriptor tables.
 *	  Usable also in assembler modules.
 */

#include <bspopts.h>

#define IDT_SIZE (256)
#define GDT_SIZE (3 + NUM_APP_DRV_GDT_DESCRIPTORS)

