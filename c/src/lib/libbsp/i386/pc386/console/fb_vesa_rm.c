/*
 *  FB driver for graphic hardware compatible with VESA Bios Extension
 *  Real mode interface utilized
 *  Tested on real HW.
 *
 *  Copyright (c) 2014 - CTU in Prague
 *                       Jan Doležal ( dolezj21@fel.cvut.cz )
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 *
 *  The code is based on following information sources:
 *    - doc/bsp_howto/bsp_howto.pdf
 *    - VESA BIOS EXTENSION (VBE) Core Function Standard, Ver: 3.0, Sep 16, 1998
 *    - RTEMS fb_vga.c - Rosimildo da Silva ( rdasilva@connecttel.com )
 *    - RTEMS fb_cirrus.c - Alexandru-Sever Horin (alex.sever.h@gmail.com)
 */

/*
 *  Hardware is completely initialized upon boot of the system.
 *  Therefore there is no way to change graphic mode.
 *  
 *  Interrupt 0x10 is used for entering graphics BIOS.
 */

#include <vbe3.h>

#include <pthread.h>
#include <string.h>

#include <bsp.h>
//#include <rtems/score/cpu.h>
#include <libcpu/cpu.h>
#include <rtems/libio.h>

#include <rtems/fb.h>
#include <rtems/framebuffer.h>
#include <rtems/asm.h>

#include <rtems/score/registers.h>

#define FB_VESA_NAME    "FB_VESA_RM"

/* mutex for protection against multiple opens, when called frame_buffer_open */
static pthread_mutex_t vesa_mutex = PTHREAD_MUTEX_INITIALIZER;

/* screen information for the VGA driver
 * standard structures - from RTEMS fb interface
 */
/*static struct fb_var_screeninfo fb_var;*/
static struct fb_fix_screeninfo fb_fix;

#define VBE_REG_LEN 22
struct VBE_registers { /* used for passing parameters and fetching results */
    uint32_t reg_eax;                           /* off 0  */
    uint32_t reg_ebx;                           /* off 4  */
    uint32_t reg_ecx;                           /* off 8  */
    uint32_t reg_edx;                           /* off c  */
    uint32_t reg_edi;                           /* off 10 */
    uint16_t reg_es;                            /* off 14 */
} __attribute__((__packed__));

/* address where are located VBE buffer, parameter/returned values and code for calling VBE real mode interface */
#define VESA_SPOT   0x1000
#define VBE_BUF_LEN 512
/* position for real mode code reallocation to the first MB of RAM */
#define VESA_FNC VESA_SPOT+VBE_BUF_LEN+VBE_REG_LEN

void vbe_realmode_if(){
        /* copy desired code to first 64kB */
    __asm__ volatile(   "\t"
        "movl    $cp_end-cp_beg, %%ecx\n\t"
        "cld\n\t"
        "movl    $cp_beg, %%esi\n\t"
        "movl    %0, %%edi\n\t"
        "rep movsb\n\t"
        /* test that paging is off */
        "movl    %%cr0, %%eax\n\t"
        "testl   %1, %%eax\n"
"pg_dis: jne     pg_dis\n\t" /* hopefuly no loader turnes on paging */
        /* jump to copied function */
        "jmp     %3\n"
        /* load 'real mode like' selectors */
"cp_beg: movw    $0x20, %%ax\n\t" /* fourth element of GDT */
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"

        /* load 'real mode like' code selector */
        "ljmp    $0x18, %0+(cs_real-cp_beg)\n"
"cs_real:"
        ".code16\n\t"
        /* disable protected mode */
        "movl    %%cr0, %%eax\n\t"
        "andl    %4, %%eax\n\t"
        "movl    %%eax, %%cr0\n\t"
        /* suppose loader does not shift or damage interrupt table on the beginning of memory; that means length: 0x3FF, base: 0x0 */

        /* flush prefetch queue by far jumping */
        /* change selector in segmentation register to correct real mode style segment */
        "ljmp    $0x0, %0+(dsels-cp_beg)\n"
"dsels:  xor     %%ax, %%ax\n\t"
        "mov     %%ax, %%ss\n\t"
        "mov     %%ax, %%ds\n\t"
        "mov     %%ax, %%es\n\t"
        "mov     %%ax, %%fs\n\t"
        "mov     %%ax, %%gs\n\t"
        
        "mov     %0+(cp_end-cp_beg), %%edi\n\t"
        /* fill registers with parameters */
        "movl    %5, %%esi\n\t"
        "movl    0x00(%%esi), %%eax\n\t"
        "movl    0x04(%%esi), %%ebx\n\t"
        "movl    0x08(%%esi), %%ecx\n\t"
        "movl    0x0c(%%esi), %%edx\n\t"
        "movl    0x10(%%esi), %%edi\n\t"
        "movw    0x14(%%esi), %%es\n\t"

        "int     $0x10\n\t"
//        "pushl   %%esi\n\t"
        "movl    %5, %%esi\n\t"
        "movl    %%eax,0x00(%%esi)\n\t"
        "movl    %%ebx,0x04(%%esi)\n\t"
        "movl    %%ecx,0x08(%%esi)\n\t"
        "movl    %%edx,0x0c(%%esi)\n\t"
        "movl    %%ebp,0x10(%%esi)\n\t"
        "movw    %%es, 0x14(%%esi)\n\t"
//        "popl    %%esi\n\t"
"aftint:"
        /* fill return structure */

        "movl    %%cr0, %%eax     \n\t"
        "orl     %2, %%eax    \n\t"
        "movl    %%eax, %%cr0     \n\t"
        "ljmpl   $0x8,$cp_end \n\t"
        ".code32\n"
"cp_end: movl    $0x10,%%ax\n\t"
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"
        : 
        : "i"(VESA_FNC), "i"(CR0_PAGING), "i"(CR0_PROTECTION_ENABLE), "r"(VESA_FNC), "i"(~CR0_PROTECTION_ENABLE), "i"(VESA_SPOT+VBE_BUF_LEN)
        : "memory", "%eax", "%ebx", "%ecx", "%edx", "%esi", "%edi"
    );

}

void vesa_realmode_bootup_init(){
    /* create 'real mode like' segment descriptors, for switching to real mode */
    #define rml_base  0x0
    #define rml_limit 0xFFFF
    uint16_t rml_code_dsc, rml_data_dsc;

    /* load GDT register with RTEMS gdt struct */
    __asm__ volatile(   "\t"
                        "lgdt gdtdesc\n\t" /* load RTEMS gdt descriptor */
                        "ljmp $0x8, $1f\n" /* assuming code segment descriptor is the first entry in GDT */
                        "1:"
                    );
    
    rml_code_dsc = i386_find_empty_gdt_entry();

    if(rml_code_dsc==0)
    {
        /* not enough space in GDT */
nsgdtc: goto nsgdtc;
        return;
    }
    
    segment_descriptors flags_desc;
    flags_desc.type                = 0xE;      /* bits 4  */
    flags_desc.descriptor_type     = 0x1;      /* bits 1  */
    flags_desc.privilege           = 0x0;      /* bits 2  */
    flags_desc.present             = 0x1;      /* bits 1  */
    flags_desc.available           = 0x0;      /* bits 1  */
    flags_desc.fixed_value_bits    = 0x0;      /* bits 1  */
    flags_desc.operation_size      = 0x0;      /* bits 1  */
    flags_desc.granularity         = 0x0;      /* bits 1  */
    if(i386_put_gdt_entry(rml_code_dsc, rml_base, rml_limit, &flags_desc)==0)
    {
orc:    goto orc; /* selector to GDT out of range */
    }

    rml_data_dsc = i386_find_empty_gdt_entry();
    if(rml_data_dsc==0)
    {
        /* not enough space in GDT */
nsgdtd: goto nsgdtd;
        return;
    }
    flags_desc.type                = 0x2;      /* bits 4  */
    if(i386_put_gdt_entry(rml_data_dsc, rml_base, rml_limit, &flags_desc)==0)
    {
ord:    goto ord; /* selector to GDT out of range */
    }

    /* structure containing parameters */
    /* after call to VBE realmode interface, there will be returned values */
    void *VBE_buffer = (void *)(VESA_SPOT);
    struct VBE_registers *parret = (struct VBE_registers *)(VESA_SPOT+VBE_BUF_LEN);

    parret->reg_eax = 0x4f00;
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;
    vbe_realmode_if();

    struct VBE_VbeInfoBlock *vib = (struct VBE_VbeInfoBlock *)VBE_buffer;
    printk("%s\n", &vib->VbeSignature);
    printk("%x\n", vib->VbeVersion);
    printk("%s\n", vib->OemStringPtr);
    printk("%x\n", vib->Capabilities);
    printk("video modes: ");
    uint16_t *modeNOPtr = vib->VideoModePtr;
    while(*modeNOPtr != 0){
        printk("%x, ", *modeNOPtr);
        modeNOPtr += sizeof(uint16_t);
    }
    printk("\n");
//    printk("%p\n", vib->VideoModePtr);
    printk("%d\n", vib->TotalMemory);
    printk("%d\n", vib->OemSoftwareRev);
    printk("%s\n", vib->OemVendorNamePtr);
    printk("%s\n", vib->OemProductNamePtr);
    printk("%s\n", vib->OemProductRevPtr);

    parret->reg_eax = 0x4f01;
    parret->reg_ecx = *(vib->VideoModePtr);
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;
    vbe_realmode_if();
    printk();

    vib = (void *) 0;
stop: goto stop;
    /* deallocate gdt entries */
    if(i386_free_gdt_entry(rml_code_dsc))
    {
sorc:   goto sorc; /* selector to GDT out of range */
    }
    if(i386_free_gdt_entry(rml_data_dsc))
    {
dorc:   goto dorc; /* selector to GDT out of range */
    }
}

/*
 * fb_vesa device driver INITIALIZE entry point.
 */
rtems_device_driver
frame_buffer_initialize(
    rtems_device_major_number  major,
    rtems_device_minor_number  minor,
    void                      *arg
)
{
    rtems_status_code status;

    printk(FB_VESA_NAME " frame buffer -- driver initializing..\n" );

/*
 * Register the device.
 */
    status = rtems_io_register_name(FRAMEBUFFER_DEVICE_0_NAME, major, 0);
    if (status != RTEMS_SUCCESSFUL)
    {
        printk("Error registering " FRAMEBUFFER_DEVICE_0_NAME
        " - " FB_VESA_NAME " frame buffer device!\n");
        rtems_fatal_error_occurred( status );
    }

    return RTEMS_SUCCESSFUL;

_error:
    printk("error\n");
    BSP_wait_polled_input();
    free_used_descriptors();
    return 2;
}


/*
 * fb_vesa device driver OPEN entry point
 */
rtems_device_driver
frame_buffer_open(
    rtems_device_major_number  major,
    rtems_device_minor_number  minor,
    void                      *arg
)
{
    printk( FB_VESA_NAME " open device\n" );

    if (pthread_mutex_trylock(&vesa_mutex)!= 0){
        printk( FB_VESA_NAME " could not lock vesa_mutex\n" );

        return RTEMS_UNSATISFIED;
    }


    return RTEMS_SUCCESSFUL;

}


/*
 * fb_vesa device driver CLOSE entry point
 */
rtems_device_driver
frame_buffer_close(
    rtems_device_major_number  major,
    rtems_device_minor_number  minor,
    void                      *arg
)
{
  printk( FB_VESA_NAME " close device\n" );
  if (pthread_mutex_unlock(&vesa_mutex) == 0){
      /* restore previous state.  for VGA this means return to text mode.
       * leave out if graphics hardware has been initialized in
       * frame_buffer_initialize() */

      printk(FB_VESA_NAME ": close called.\n" );
      return RTEMS_SUCCESSFUL;
  }

  return RTEMS_UNSATISFIED;
}


/*
 * fb_vesa device driver READ entry point.
 */
rtems_device_driver
frame_buffer_read(
    rtems_device_major_number  major,
    rtems_device_minor_number  minor,
    void                      *arg
)
{
  printk( FB_VESA_NAME " read device\n" );
  rtems_libio_rw_args_t *rw_args = (rtems_libio_rw_args_t *)arg;
  rw_args->bytes_moved = ((rw_args->offset + rw_args->count) > fb_fix.smem_len ) ? (fb_fix.smem_len - rw_args->offset) : rw_args->count;
  memcpy(rw_args->buffer, (const void *) (fb_fix.smem_start + rw_args->offset), rw_args->bytes_moved);
  return RTEMS_SUCCESSFUL;
}


/*
 * frame_vesa device driver WRITE entry point.
 */
rtems_device_driver
frame_buffer_write(
    rtems_device_major_number  major,
    rtems_device_minor_number  minor,
    void                      *arg
)
{
  printk( FB_VESA_NAME " write device\n" );
  rtems_libio_rw_args_t *rw_args = (rtems_libio_rw_args_t *)arg;
  rw_args->bytes_moved = ((rw_args->offset + rw_args->count) > fb_fix.smem_len ) ? (fb_fix.smem_len - rw_args->offset) : rw_args->count;
  memcpy( (void *) (fb_fix.smem_start + rw_args->offset), rw_args->buffer, rw_args->bytes_moved);
  return RTEMS_SUCCESSFUL;
}


/*
 * IOCTL entry point -- This method is called to carry
 * all services of this interface.
 */
rtems_device_driver
frame_buffer_control(
    rtems_device_major_number  major,
    rtems_device_minor_number  minor,
    void                      *arg
)
{
  rtems_libio_ioctl_args_t *args = arg;

  printk( FB_VESA_NAME " ioctl called, cmd=%x\n", args->command  );

  switch( args->command ) {
  case FBIOGET_FSCREENINFO:
    /* not implemented yet */
    args->ioctl_return = -1; 
    return RTEMS_UNSATISFIED;
    break;
  case FBIOGET_VSCREENINFO:
    /* not implemented yet */
    args->ioctl_return = -1;
    return RTEMS_UNSATISFIED;
    break;
  case FBIOPUT_VSCREENINFO:
    /* not implemented yet */
    args->ioctl_return = -1;
    return RTEMS_UNSATISFIED;
  case FBIOGETCMAP:
    /* no palette - truecolor mode */
    args->ioctl_return = -1;
    return RTEMS_UNSATISFIED;
  case FBIOPUTCMAP:
    /* no palette - truecolor mode */
    args->ioctl_return = -1;
    return RTEMS_UNSATISFIED;
  default:
    args->ioctl_return = 0;
    break;
  }
  return RTEMS_SUCCESSFUL;
}
