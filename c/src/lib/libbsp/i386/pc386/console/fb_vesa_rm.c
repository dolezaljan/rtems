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

#define VBE_REG_LEN 0x20
struct VBE_registers { /* used for passing parameters, fetching results and preserving values */
    uint32_t reg_eax;                           /* off 0x0  */
    uint32_t reg_ebx;                           /* off 0x4  */
    uint32_t reg_ecx;                           /* off 0x8  */
    uint32_t reg_edx;                           /* off 0xC  */
    uint32_t reg_edi;                           /* off 0x10 */
    uint16_t reg_es;                            /* off 0x14 */
    uint32_t reg_esp_bkp;                       /* off 0x16 */
    uint16_t idtr_lim_bkp;                      /* off 0x1A */
    uint32_t idtr_base_bkp;                     /* off 0x1C */
    /* if adding new element update VBE_REG_LEN as well */
} __attribute__((__packed__));

/* address where we are going to put VBE buffer, parameter/returned/preserved values and code for calling VBE real mode interface */
#define VESA_SPOT   0x1000
#define VBE_BUF_SPOT VESA_SPOT
#define VBE_BUF_LEN 512
#define VBE_REGS_SPOT VBE_BUF_SPOT+VBE_BUF_LEN
/* position for real mode code reallocation to the first MB of RAM */
#define VESA_FNC_SPOT VBE_REGS_SPOT+VBE_REG_LEN
#define VBE_STACK_TOP VESA_FNC_SPOT+0x500

/******************************
 * VBE_BUF          * 512 B   *
 ******************************
 * VBE_REGs         * 32 B    *
 ******************************
 * VESA_FNC         *         *
 ******************** 0x500 B *
 * STACK            *         *
 ******************************/

extern int BSP_wait_polled_input(); /* for debugging purposes */

/**
 * This function presumes prepared real mode like descriptors for code (index 4 - selector 0x20) and data (index 3 - selector 0x18) in the GDT.
 */
static void vbe_realmode_if(){
        /* copy desired code to first 64kB of RAM */
    __asm__ volatile(   "\t"
        "movl    $cp_end-cp_beg, %%ecx\n\t"
        "cld\n\t"
        "movl    $cp_beg, %%esi\n\t"
        "movl    %0, %%edi\n\t"
        "rep movsb\n\t"
        /* test that paging is off */
        "movl    %%cr0, %%eax\n\t"
        "testl   %3, %%eax\n"
"pg_dis: jne     pg_dis\n\t" /* hopefuly no loader turns on paging */
        "cli\n\t"
        /* jump to copied function */
        "movl    %0, %%eax\n\t"
        "jmp     %%eax\n"
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
        "andl    %5, %%eax\n\t"
        "movl    %%eax, %%cr0\n\t"
        /* hopefully loader does not damage interrupt table on the beginning of memory; that means length: 0x3FF, base: 0x0 */
        /* XXX: consider preserving idtr */
//        "movl    %1+0x1A, %%eax\n\t"
//        "sidt    (%%eax)\n\t"
        "movl    %0+(begidt-cp_beg), %%eax\n\t"
        "lidt    (%%eax)\n\t"
        /* flush prefetch queue by far jumping */
        /* change selector in segmentation register to correct real mode style segment */
        "ljmp    $0x0, %0+(dsels-cp_beg)\n"
        /* limit and base for realmode interrupt descriptor table */
"begidt:"
        ".word 0x3FF\n\t"
        ".long 0\n\t"
"dsels:  xor     %%ax, %%ax\n\t"
        "mov     %%ax, %%ss\n\t"
        "mov     %%ax, %%ds\n\t"
        "mov     %%ax, %%fs\n\t"
        "mov     %%ax, %%gs\n\t"
        /* fill registers with parameters */
        "movl    %1, %%esi\n\t"
        "movl    0x00(%%esi), %%eax\n\t"
        "movl    0x04(%%esi), %%ebx\n\t"
        "movl    0x08(%%esi), %%ecx\n\t"
        "movl    0x0c(%%esi), %%edx\n\t"
        "movl    0x10(%%esi), %%edi\n\t"
        "movw    0x14(%%esi), %%es\n\t"
        /* backup stack pointer */
        "movl    %%esp, 0x16(%%esi)\n\t"
        /* establish rm stack */
        "movl    %2, %%esp\n\t"
        "int     $0x10\n\t"

        /* fill return structure */
        "movl    %1, %%esi\n\t"
        "movl    %%eax,0x00(%%esi)\n\t"
        "movl    %%ebx,0x04(%%esi)\n\t"
        "movl    %%ecx,0x08(%%esi)\n\t"
        "movl    %%edx,0x0c(%%esi)\n\t"
        "movl    %%ebp,0x10(%%esi)\n\t"
        "movw    %%es, 0x14(%%esi)\n\t"
        /* restore original stack pointer */
        "movl    0x16(%%esi),%%esp\n\t"
"aftint:"
        /* return to protected mode */
        "movl    %%cr0, %%eax     \n\t"
        "orl     %4, %%eax    \n\t"
        "movl    %%eax, %%cr0     \n\t"
        "ljmpl   $0x8,$cp_end \n\t"
        ".code32\n"
        /* reload segmentation registers */
"cp_end: movw    $0x10,%%ax\n\t"
        "movw    %%ax, %%ss\n\t"
        "movw    %%ax, %%ds\n\t"
        "movw    %%ax, %%es\n\t"
        "movw    %%ax, %%fs\n\t"
        "movw    %%ax, %%gs\n\t"
        /* restore IDTR */
//        "movl    %1+0x1A, %%eax\n\t"
//        "lidt    (%%eax)\n\t"
        : 
        : "i"(VESA_FNC_SPOT), "i"(VBE_REGS_SPOT), "i"(VBE_STACK_TOP), "i"(CR0_PAGING), "i"(CR0_PROTECTION_ENABLE), "i"(~CR0_PROTECTION_ENABLE)
        : "memory", "eax", "ebx", "ecx", "edx", "esi", "edi"
    );
}

void vesa_realmode_bootup_init(){
    /* create 'real mode like' segment descriptors, for switching to real mode */
    #define rml_base  0x0
    #define rml_limit 0xFFFF
    uint16_t rml_code_dsc, rml_data_dsc;

    /* load GDT register with RTEMS gdt struct and reload segment registers */
    __asm__ volatile(   "\t"
                        "lgdt gdtdesc\n\t" /* load RTEMS gdt descriptor */
                        "ljmp $0x8, $1f\n" /* assuming code segment descriptor is the first entry in GDT */
                        "1:\t"
                        "movw $0x10, %%ax\n\t" /* assuming data segment descriptor is the second entry in GDT */
                        "movw %%ax, %%ds\n\t"
                        "movw %%ax, %%es\n\t"
                        "movw %%ax, %%fs\n\t"
                        "movw %%ax, %%gs\n\t"
                        "movw %%ax, %%ss\n\t"
                        :
                        :
                        : "eax"
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
    void *VBE_buffer = (void *)(VBE_BUF_SPOT);
    struct VBE_registers *parret = (struct VBE_registers *)(VBE_REGS_SPOT);

    parret->reg_eax = VBE_RetVBEConInf;
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;

    vbe_realmode_if();

    /* check success of function call */
        printk("returned eax=0x%x\n", parret->reg_eax);
    if((parret->reg_eax&0xff)!=VBE_functionSupported || (parret->reg_eax&0xff00)!=VBE_callSuccessful){
        printk("Function 00h not supported. eax=0x%x\n", parret->reg_eax);
    }

/* see VBE CORE FUNCTIONS VERSION 3.0 Pag.65 - Appendix 1 - VBE Implementation Considerations */
#define VBE_STUB_VideoModeList 0xFFFF
    struct VBE_VbeInfoBlock *vib = (struct VBE_VbeInfoBlock *)VBE_buffer;
    if(*(uint16_t*)vib->VideoModePtr == VBE_STUB_VideoModeList)
    {
        printk("VBE Core not implemented!\n");
    }
    printk("Signature: %s\n", &vib->VbeSignature);
    printk("VBE Ver  :%x\n", vib->VbeVersion);
    printk("OemString:%s\n", vib->OemStringPtr);
    printk("Capabilit:%x\n", vib->Capabilities);
    printk("video modes: ");
    uint16_t *modeNOPtr = (uint16_t*)vib->VideoModePtr;
    uint16_t iterator = 0;
    uint16_t VideoModes[100];
    while(*(modeNOPtr+iterator) != 0){
        *(VideoModes+iterator) = *(modeNOPtr+iterator);
        printk("%x, ", *(VideoModes+iterator));
        iterator += sizeof(uint16_t);
    }
    printk("\n");
    printk("TotMemory:%d\n", vib->TotalMemory);
    printk("OemSwRev :%d\n", vib->OemSoftwareRev);
    printk("OemVenNam:%s\n", vib->OemVendorNamePtr);
    printk("OemProdNm:%s\n", vib->OemProductNamePtr);
    printk("OemProRev:%s\n", vib->OemProductRevPtr);

    BSP_wait_polled_input();

    printk("VBE/DDC capabilities:\n");
    parret->reg_eax = VBE_DisDatCha;
    parret->reg_ebx = VBEDDC_Capabilities;
    parret->reg_ecx = 0; /* controller unit nr. */
    parret->reg_edi = 0;
    parret->reg_es = 0;
    vbe_realmode_if();
    /* check success of function call */
        printk("returned eax=0x%x\n", parret->reg_eax);
    if((parret->reg_eax&0xff)!=VBE_functionSupported || (parret->reg_eax&0xff00)!=VBE_callSuccessful){
        printk("Function 15h not supported. eax=0x%x\n", parret->reg_eax);
    }
    printk("It takes approximately %d seconds to transfer one EDID block (128 bytes).\n", parret->reg_ebx>>8&0xff);
    printk("DDC level supported: %x\n",parret->reg_ebx&0xff);
    

    BSP_wait_polled_input();
    printk("Read E-EDID through VBE/DDC\n");
    //struct EDID *edid = (struct EDID *)VBE_buffer;
    parret->reg_eax = VBE_DisDatCha;
    parret->reg_ebx = VBEDDC_ReadEDID;
    parret->reg_ecx = 0; /* controller unit nr. */
    parret->reg_edx = 0; /* EDID block nr. */
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;
    vbe_realmode_if();
    /* check success of function call */
        printk("returned eax=0x%x\n", parret->reg_eax);
    if((parret->reg_eax&0xff)!=VBE_functionSupported || (parret->reg_eax&0xff00)!=VBE_callSuccessful){
        printk("Function 15h not supported. eax=0x%x\n", parret->reg_eax);
    }

    iterator = 0;
    struct VBE_ModeInfoBlock *mib = (struct VBE_ModeInfoBlock *)VBE_buffer;

    while(VideoModes[iterator]!=0){
        BSP_wait_polled_input();
        printk("Mode: %x\n", VideoModes[iterator]);
        parret->reg_eax = VBE_RetVBEModInf;
        parret->reg_ecx = VideoModes[iterator];
        parret->reg_edi = (uint32_t)VBE_buffer;
        parret->reg_es = 0x0;
        vbe_realmode_if();
        printk("returned eax=0x%x\n", parret->reg_eax);
        printk("ModeAttributes: %x\n", mib->ModeAttributes);
        printk("WinAAttributes: %x, WinBAttributes: %x\n", mib->WinAAttributes, mib->WinBAttributes);
        printk("WinGranularity: %x, WinSize: %x\n", mib->WinGranularity, mib->WinSize);
        printk("WinASegment: %x, WinBSegment: %x\n", mib->WinASegment, mib->WinBSegment);
        printk("WinFuncPtr: %p, BytesPerScanLine: %x\n", mib->WinFuncPtr, mib->BytesPerScanLine);
        printk("XResolution: %d, YResolution: %d\n", mib->XResolution, mib->YResolution);
        printk("XCharSize: %dpx, YCharSize: %dpx\n", mib->XCharSize, mib->YCharSize);
        printk("NumberOfPlanes: %d\n", mib->NumberOfPlanes);
        printk("BitsPerPixel: %d\n", mib->BitsPerPixel);
        printk("NumberOfBanks: %d\n", mib->NumberOfBanks);
        printk("MemoryModel: %d\n", mib->MemoryModel);
        printk("BankSize: %dKB\n", mib->BankSize);
        printk("NumberOfImagePages: %d\n", mib->NumberOfImagePages);
        printk("Reserverd: %x", mib->Reserved0);
        printk("...\nPhysBasePtr: 0x%p -- physical address for flat memory frame buffer\n...\n\n", mib->PhysBasePtr);
        iterator += sizeof(uint16_t);
    }

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

//_error:
//    printk("error\n");
//    BSP_wait_polled_input();
//    free_used_descriptors();
//    return 2;
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
