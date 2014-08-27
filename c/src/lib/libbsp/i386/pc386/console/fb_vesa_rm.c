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

#define FB_VESA_NAME    "FB_VESA"
#define SEG_16b_LEN     0x10000 /* top length of 16-bit segments in bytes */
#define VID_MEM_SIZE    0x10000 /* length in bytes */
#define VIDEO_MEM_GR    0xA0000 /* physical address of the beginning of video memory for vga graphics modes */
#define VIDEO_MEM_TXT_B 0xB0000 /* physical address of the beginning of video memory for text modes */
#define VIDEO_MEM_TXT_C 0xB8000 /* physical address of the beginning of video memory for coloured text modes */
#define GRBIOS_HW_ADDR  0xC0000 /* physical address of the beginning of video BIOS */
#define GRBIOS_SIZE     0x10000 /* although according to VBE spec PMID struc should be located within
                                   the first 32kB of the BIOS, implementations having PMID struc further
                                   in the ROM have been encountered */
#define GRBIOS_DATAEMUL 0x600   /* at least 600h */
#define GRBIOS_STACK_LN 0x400   /* at least 1024 bytes */

/* mutex for protection against multiple opens, when called frame_buffer_open */
static pthread_mutex_t vesa_mutex = PTHREAD_MUTEX_INITIALIZER;

/* screen information for the VGA driver
 * standard structures - from RTEMS fb interface
 */
/*static struct fb_var_screeninfo fb_var;*/
static struct fb_fix_screeninfo fb_fix;

/* space for the copy of graphics BIOS */
uint8_t vbe_grbios_img[GRBIOS_SIZE];
uint8_t vbe_grbios_dataemul[GRBIOS_DATAEMUL];
uint16_t vbe_grbios_stack[GRBIOS_STACK_LN/sizeof(uint16_t)];

/* this struct is intended to hold indexes to GDT table (range 0-65535), not segment selectors itself */
static struct {
    uint16_t code;
    uint16_t dataEmul;
    uint16_t A;
    uint16_t B;
    uint16_t B8;
    uint16_t dataCode;
    uint16_t stack;
    uint16_t parameters;
    uint16_t db_switch;
} used_segment_selectors;

static struct VBE_PMInfoBlock* pmib;

/* struct for parameters for graphical bios */
struct VBE_parameters {
    uint32_t call_eax;                      /* off 0  - function */
    uint32_t call_ebx;                      /* off 4  - subfunction/mode */
    uint32_t call_ecx;                      /* off 8  */
    uint32_t call_edx;                      /* off c  */
    uint32_t call_edi;                      /* off 10 - offset of tables/buffers */
    uint16_t call_es;                       /* off 14 - segment of tables/buffers */
    uint16_t vbe_offset;                    /* off 16 */
    uint16_t vbe_cs;                        /* off 18 */
} __attribute__((__packed__));

/* struct for return values from graphical bios */
struct VBE_return {
    uint32_t reg_eax;                           /* off 0  */
    uint32_t reg_ebx;                           /* off 4  */
    uint32_t reg_ecx;                           /* off 8  */
    uint32_t reg_edx;                           /* off c  */
    uint32_t reg_edi;                           /* off 10 */
    uint16_t reg_es;                            /* off 14 */
} __attribute__((__packed__));

void vbe_realmode_if(){


}

void vesa_realmode_bootup_init(){
    /* create 'real mode like' segment descriptors, for switching to real mode */
    #define rml_base  0x0
    #define rml_limit 0xFFFF
    uint16_t rml_code_dsc, rml_data_dsc;

    /* load GDT register with RTEMS gdt struct */
    __asm__ volatile(   "\t"
                        "lgdt gdtdesc\n\t" /* load RTEMS gdt descriptor */
                        "ljmp $0x8, $1f\n" /* we assume, code segment descriptor is the first entry in GDT */
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

    /* test call to real mode interrupt */
    vbe_realmode_if();

    /* deallocate gdt entries */
//    if(i386_free_gdt_entry(rml_code_dsc))
//    {
//sorc:   goto sorc; /* selector to GDT out of range */
//    }
//    if(i386_free_gdt_entry(rml_data_dsc))
//    {
//dorc:   goto dorc; /* selector to GDT out of range */
//    }
}

/* 
 * free GDT from used descriptors
 */
void free_used_descriptors()
{
    uint32_t index;
    uint16_t segment_selector;
    for(index=0;index<sizeof(used_segment_selectors)/sizeof(uint16_t);index++)
    {
        segment_selector = *(((uint16_t*)&used_segment_selectors)+index);
        if(segment_selector)
        {
            i386_free_gdt_entry(segment_selector);
            *(((uint16_t*)&used_segment_selectors)+index) = 0;
        }
    }
}

uint16_t prepareESdescriptor(uint32_t base, uint32_t size)
{
  segment_descriptors flags_desc;
  uint16_t currentDSselector;
  __asm__ volatile ("\tmov %%ds,%0" : "=r" (currentDSselector) );
  uint32_t currentBase = i386_base_gdt_entry(i386_get_gdt_entry(currentDSselector>>3));
  flags_desc.type                = 0x2;      /* bits 4  */
  flags_desc.descriptor_type     = 0x1;      /* bits 1  */
  flags_desc.privilege           = 0x0;      /* bits 2  */
  flags_desc.present             = 0x1;      /* bits 1  */
  flags_desc.available           = 0x0;      /* bits 1  */
  flags_desc.fixed_value_bits    = 0x0;      /* bits 1  */
  flags_desc.operation_size      = 0x0;      /* bits 1  */
  if(!used_segment_selectors.parameters){
    used_segment_selectors.parameters = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.parameters)
    {
        printk(FB_VESA_NAME "(parameters): not enough free descriptors in GDT\n");
        return 0;
    }
  }
  i386_put_gdt_entry (used_segment_selectors.parameters, currentBase+base, size-1, &flags_desc);
  return used_segment_selectors.parameters << 3;
}

uint32_t vbe_function_call(struct VBE_parameters* params, struct VBE_return* returns)
{
    struct {
        uint16_t return_cs;                     /* off 0 */
        uint16_t call_ss;                       /* off 2 */
        uint32_t size_gr_bios_stack;            /* off 4 */
        uint32_t backup_esp;                    /* off 8 */
        uint16_t backup_ss;                     /* off c */
        uint32_t base_gr_bios_stack;            /* off e */
        uint32_t base_vbe_cs;                   /* off 12*/
        uint32_t size_vbe_cs;                   /* off 16*/
    } __attribute__((__packed__)) _locals, *locals = &_locals;

    /* create return segment selector and fill respective descriptor in GDT */
    segment_descriptors flags_desc;
    flags_desc.type                = 0xA;      /* bits 4  */
    flags_desc.descriptor_type     = 0x1;      /* bits 1  */
    flags_desc.privilege           = 0x0;      /* bits 2  */
    flags_desc.present             = 0x1;      /* bits 1  */
    flags_desc.available           = 0x0;      /* bits 1  */
    flags_desc.fixed_value_bits    = 0x0;      /* bits 1  */
    flags_desc.operation_size      = 0x1;      /* bits 1  */
    if(!used_segment_selectors.db_switch)
    {
        used_segment_selectors.db_switch = i386_find_empty_gdt_entry();
        if(!used_segment_selectors.db_switch)
        {
            printk(FB_VESA_NAME " (db_switch): not enough free descriptors in GDT\n");
            return RTEMS_UNSATISFIED;
        }
    }
    uint32_t return_base;
    __asm__ volatile ("\tmovl $return32_seg,%[base]" : [base] "=m" (return_base)  );
    i386_put_gdt_entry (used_segment_selectors.db_switch, return_base, 0xFFFFFFFF, &flags_desc);
    _locals.return_cs = used_segment_selectors.db_switch << 3;
    _locals.call_ss   = used_segment_selectors.stack     << 3; /* using already filled descriptor */
    _locals.size_gr_bios_stack = sizeof(vbe_grbios_stack);
    _locals.base_gr_bios_stack = vbe_grbios_stack;

    segment_descriptors* vbe_cs_ptr = i386_get_gdt_entry(params->vbe_cs);
    _locals.base_vbe_cs = i386_base_gdt_entry(vbe_cs_ptr);
    _locals.size_vbe_cs = GRBIOS_SIZE;

/*************testing*********************/
//    uint32_t adrrr; // testing return from subsegment
//    __asm__ volatile ("\tmovl $nlnretur,%[add]" : [add] "=m" (adrrr) );
//    params->vbe_offset = 0;
//    flags_desc.operation_size = 0;
//    i386_put_gdt_entry (used_segment_selectors.code, adrrr, GRBIOS_SIZE-1, &flags_desc);
/*************end testing*********************/
    printk("asm\n");
    __asm__ __volatile__  (
                    "\t"
                    "pushw %%ds\n\t"
                    "pushw %%es\n\t"
                    "pushw %%fs\n\t"
                    "pushw %%gs\n\t"
                    "pushl %%ebp\n\t"
                    "pushfl\n\t"
                    "pushl %%edi\n\t"
                    /* for return to plain 32-bit segment */
                    "pushl %%cs\n\t"
                    "pushl $end_vbe_fnc\n\t"
                    /* backup current stack */
                    "movw %%ss,0xc(%%esi)\n\t"
                    "movl %%esp,0x8(%%esi)\n\t"
                    "pushl %%ecx\n\t"
                    /* backup pointer to locals (ds:esi) */
                    "movl 0xe(%%esi),%%ebx\n\t"
                    "movl 0x4(%%esi),%%edi\n\t"
                    "movl %%esi,-0x4(%%ebx,%%edi)\n\t"
                    "movw %%ds,-0x6(%%ebx,%%edi)\n\t"
                    /* for return to 32-bit segment with offset certainly less than 0xFFFF */
                    /* store return information to the end of graphical bios stack */
                    "movw (%%esi),%%ax\n\t"
                    "movw %%ax,-0x8(%%ebx,%%edi)\n\t"
                    "movw $0,-0xa(%%ebx,%%edi)\n\t"
                    /* disable interrupts (must be disabled as long as segmentation registers are not held per task) */
                    "cli\n\t"
                    /* clear direction flag */
                    "cld\n\t"
                    /* load new stack - used for graphical bios */
                    "popl %%ebp\n\t" /* guessing ebp is not ever used as parameter for vbe function */
                    "movw 0x2(%%esi),%%ss\n\t"
                    "movl $0xFFF6,%%esp\n\t" /* there is stack segment and offset prepared on the end of the stack for return and there is also pointer to local parameters at the very top */
                    /* prepare parameters to registers */
                    "movl %%ebp,%%esi\n\t"
                    "movl 0x00(%%esi),%%eax\n\t"
                    "movl 0x04(%%esi),%%ebx\n\t"
                    "movl 0x08(%%esi),%%ecx\n\t"
                    "movl 0x0c(%%esi),%%edx\n\t"
                    "movl 0x10(%%esi),%%edi\n\t"
                    "movw 0x14(%%esi),%%es\n\t"
                    
                    /* jump to the graphical bios code */
                    "data16 ljmp *0x16(%%esi)\n\t"

                    "return32_seg:\n\t"
                    /* gain address of locals */
                    "movw (%%esp),%%ds\n\t"
                    "movl 0x2(%%esp),%%ebp\n\t" /* guessing ebp is not ever used for return value from vbe function */
                    "movw %%ds:0xc(%%ebp),%%ss\n\t"
                    "movl %%ds:0x8(%%ebp),%%esp\n\t"
                    "lret\n"
/*************testing*********************/
//                    "nlnretur:\n\t"
//                    "addr16 lret\n"
/*************end testing*********************/

                    "end_vbe_fnc:\n\t"
                    "movl %%edi,%%ebp\n\t"
                    "popl %%edi\n\t"
                    "movl %%eax,0x00(%%edi)\n\t"
                    "movl %%ebx,0x04(%%edi)\n\t"
                    "movl %%ecx,0x08(%%edi)\n\t"
                    "movl %%edx,0x0c(%%edi)\n\t"
                    "movl %%ebp,0x10(%%edi)\n\t"
                    "movw %%es, 0x14(%%edi)\n\t"
                    /* enable interrupts */
                    "popfl\n\t"
                    "popl  %%ebp\n\t"
                    "popw  %%gs\n\t"
                    "popw  %%fs\n\t"
                    "popw  %%es\n\t"
                    "popw  %%ds\n\t"
                    :
                    : "c" (params), "S" (locals), "D" (returns)
                    : "memory", "%eax", "%ebx", "%edx"
                );
    
    return RTEMS_SUCCESSFUL;
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

    /* when FPO optimization used, local variables are addressed relatively to ESP, therefore it is not possible to use
     * push in inline assembler and access local variables - gcc does not expect this
     * solution is to use global variables, use static variables or turn off FPO */
    static volatile uint16_t ss_backup;
    static volatile uint32_t esp_backup;
    static volatile uint16_t stack16;

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

    /*
     * graphics hardware initialization
     */
    volatile uint8_t*  biosStart = (uint8_t *)GRBIOS_HW_ADDR;
    volatile void*     biosStart_mapped;
    volatile void*     vid_mem_gr_map;
    volatile void*     vid_mem_txtb_map;
    volatile void*     vid_mem_txtc_map;
    uint32_t  retVal;
    uint16_t  segment_selector;
    segment_descriptors flags_desc;
    
//    if(init_paging())
//    {
//        printk(FB_VESA_NAME ": error initializing paging\n");
//        return RTEMS_UNSATISFIED;
//    }

//    if(_CPU_map_phys_address(&biosStart_mapped, GRBIOS_HW_ADDR, GRBIOS_SIZE, 0x0)) /* flag 0x10 turns off caching */
//    {
//        printk(FB_VESA_NAME ": error mapping graphical BIOS to virtual memory\n");
//        return RTEMS_UNSATISFIED;
//    }
    biosStart_mapped = GRBIOS_HW_ADDR;

    memcpy((void*)vbe_grbios_img, biosStart_mapped, GRBIOS_SIZE-1);
//    if(_CPU_unmap_virt_address(biosStart_mapped, GRBIOS_SIZE))
//    {
//        printk(FB_VESA_NAME ": error unmapping graphical BIOS from virtual memory\n");
//    }
    biosStart_mapped = NULL;

//    pmib = (struct VBE_PMInfoBlock*) findPMID((void*)vbe_grbios_img, GRBIOS_SIZE);
    if((void *)pmib == NULL)
    {
        printk("support for VBE3_PMI not found\n");
        return RTEMS_UNSATISFIED;
    }

    /* Filling PMInfoBlock struct */
//    vbe_grbios_dataemul /* must be empty - filled with zeros */
    flags_desc.type                = 0x2;      /* bits 4  */
    flags_desc.descriptor_type     = 0x1;      /* bits 1  */
    flags_desc.privilege           = 0x0;      /* bits 2  */
    flags_desc.present             = 0x1;      /* bits 1  */
    flags_desc.available           = 0x0;      /* bits 1  */
    flags_desc.fixed_value_bits    = 0x0;      /* bits 1  */
    flags_desc.operation_size      = 0x0;      /* bits 1  */
    used_segment_selectors.dataEmul = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.dataEmul)
    {
        printk(FB_VESA_NAME "(dataEmul): not enough free descriptors in GDT\n");
        goto _error;
    }
    i386_put_gdt_entry (used_segment_selectors.dataEmul, vbe_grbios_dataemul, GRBIOS_DATAEMUL-1, &flags_desc);
    pmib->BIOSDataSel = used_segment_selectors.dataEmul << 3;

    /* next four selectors being prepared has the same flags as the previous */
//    while mapping unidentified logical error occurs, while writing, whe don't get response on display
//    if(_CPU_map_phys_address(&vid_mem_gr_map, VIDEO_MEM_GR, VID_MEM_SIZE, 0x0)) /* flag 0x10 turns off caching */
//    {
//        printk("error mapping graphical video memory to virtual memory\n");
//        return RTEMS_UNSATISFIED;
//    }
    vid_mem_gr_map = VIDEO_MEM_GR; /* workaround */

    used_segment_selectors.A = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.A)
    {
        printk(FB_VESA_NAME "(A): not enough free descriptors in GDT\n");
        goto _error;
    }
    i386_put_gdt_entry (used_segment_selectors.A, vid_mem_gr_map, VID_MEM_SIZE-1, &flags_desc);
    pmib->A0000Sel = used_segment_selectors.A << 3;
    
//    while mapping unidentified logical error occurs, while writing, whe don't get response on display
//    if(_CPU_map_phys_address(&vid_mem_txtb_map, VIDEO_MEM_TXT_B, VID_MEM_SIZE, 0x0)) /* flag 0x10 turns off caching, 0x8 write through */
//    {
//        printk("error mapping text video memory to virtual memory\n");
//        return RTEMS_UNSATISFIED;
//    }
    vid_mem_txtb_map = VIDEO_MEM_TXT_B; /* workaround */

    used_segment_selectors.B = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.B)
    {
        printk(FB_VESA_NAME "(B): not enough free descriptors in GDT\n");
        goto _error;
    }
    i386_put_gdt_entry (used_segment_selectors.B, vid_mem_txtb_map, VID_MEM_SIZE-1, &flags_desc);
    pmib->B0000Sel = used_segment_selectors.B << 3;
    
//    while mapping unidentified logical error occurs, while writing, whe don't get response on display
//    if(_CPU_map_phys_address(&vid_mem_txtc_map, VIDEO_MEM_TXT_C, VID_MEM_SIZE/2, 0x18)) /* flag 0x10 turns off caching, 0x8 write through  */
//    {
//        printk("error mapping coloured text video memory to virtual memory\n");
//        return RTEMS_UNSATISFIED;
//    }
    vid_mem_txtc_map = VIDEO_MEM_TXT_C; /* workaround */

    used_segment_selectors.B8 = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.B8)
    {
        printk(FB_VESA_NAME "(B8): not enough free descriptors in GDT\n");
        goto _error;
    }
    i386_put_gdt_entry (used_segment_selectors.B8, vid_mem_txtc_map, VID_MEM_SIZE/2-1, &flags_desc);
    pmib->B8000Sel = used_segment_selectors.B8 << 3;
    
    used_segment_selectors.dataCode = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.dataCode)
    {
        printk(FB_VESA_NAME "(dataCode): not enough free descriptors in GDT\n");
        goto _error;
    }
    i386_put_gdt_entry (used_segment_selectors.dataCode, vbe_grbios_img, GRBIOS_SIZE-1, &flags_desc);
    pmib->CodeSegSel = used_segment_selectors.dataCode << 3;

    pmib->InProtectMode = 1;

    int fidx;
    uint8_t chckSum = 0;
    for(fidx=0;fidx<sizeof(*pmib)-1;fidx++)
    {
        chckSum += *((uint8_t*)pmib+fidx);
    }
    pmib->Checksum = (~chckSum)+1;

    flags_desc.type                = 0xA;      /* bits 4  */
    used_segment_selectors.code = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.code)
    {
        printk(FB_VESA_NAME "(code): not enough free descriptors in GDT\n");
        goto _error;
    }
    i386_put_gdt_entry (used_segment_selectors.code, vbe_grbios_img, GRBIOS_SIZE-1, &flags_desc);
    
    flags_desc.type                = 0x6;      /* bits 4 - expand down toggled */
    used_segment_selectors.stack = i386_find_empty_gdt_entry();
    if(!used_segment_selectors.stack)
    {
        printk(FB_VESA_NAME " (stack): not enough free descriptors in GDT\n");
        goto _error;
    }
    /* stack grows down */
    /* offset is valid from limit+1 to 0xFFFF, for 16-bit segment */
    /* bottom of the stack is at the end of the segment - offset 0xFFFF*/
    /* by putting SP=0, first push means putting 8-bit value at offset 0xFFFF */
    /*                                          16-bit value at offset 0xFFFE */
    /*                                          32-bit value at offset 0xFFFC */
    i386_put_gdt_entry (used_segment_selectors.stack,
                        vbe_grbios_stack-(SEG_16b_LEN-sizeof(vbe_grbios_stack))/sizeof(*vbe_grbios_stack),
                        SEG_16b_LEN-GRBIOS_STACK_LN-1,
                        &flags_desc);

    struct VBE_parameters params;
    struct VBE_return returns;
    params.vbe_cs    = used_segment_selectors.code      << 3;
    params.vbe_offset = (pmib->PMInitialize);

    printk( FB_VESA_NAME " calling vbe if - pminit\n");
    if(vbe_function_call(&params, &returns) == RTEMS_UNSATISFIED)
        return RTEMS_UNSATISFIED;
    printk( FB_VESA_NAME " vbe if returned - pminit\n");

    printk("end\n");
    BSP_wait_polled_input();
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
    uint32_t ret;
    uint32_t mem_start;
    printk( FB_VESA_NAME " open device\n" );

    if (pthread_mutex_trylock(&vesa_mutex)!= 0){
        printk( FB_VESA_NAME " could not lock vesa_mutex\n" );

        return RTEMS_UNSATISFIED;
    }

    struct VBE_parameters locals;
    struct VBE_return vberet;

    locals.vbe_cs    = used_segment_selectors.code      << 3;
    locals.vbe_offset = (pmib->EntryPoint);

    locals.call_eax = 0x4f00;
    locals.call_eax |= VBE_RetVBEConInf;
//    locals.call_ebx = VBE_R1280x1024C17M;
//    locals.call_ebx |= VBE_linearFlatFrameBufMask;// | VBE_preserveDispMemMask;
    struct VBE_VbeInfoBlock vbeib;
    char Signature[] = "VBE2";
    memcpy(&vbeib.VbeSignature, &Signature, 4);
    locals.call_es = prepareESdescriptor(&vbeib, sizeof(vbeib));
    locals.call_edi = 0;

    printk( FB_VESA_NAME " calling vbe if\n");
    if(vbe_function_call(&locals, &vberet) == RTEMS_UNSATISFIED)
        return RTEMS_UNSATISFIED;
    printk( FB_VESA_NAME " vbe if returned\n");

    if( (vberet.reg_eax&0xFF) != VBE_functionSupported )
    {
      printk("VBE function is not supported\n");
      return RTEMS_UNSATISFIED;
    }
    if( ((vberet.reg_eax&0xFF00)>>8) != VBE_callSuccessful)
    {
      printk("VBE function call failed, returned: %d\n", (vberet.reg_eax>>8) );
      return RTEMS_UNSATISFIED;
    }

    printk("\n%s\nVENDOR: %p\nVBEVersion:%x\n",vbeib.VbeSignature,vbeib.OemStringPtr,vbeib.VbeVersion);
    printk("C[0]:%u C[1]:%u C[2]:%u C[3]:%u\n",vbeib.Capabilities[0],vbeib.Capabilities[1],vbeib.Capabilities[2],vbeib.Capabilities[3]);
    printk("SwRev:%x \nVenNam: %p \n ProdNam: %p \n ProdRev: %p\n",vbeib.OemSoftwareRev,vbeib.OemVendorNamePtr,vbeib.OemProductNamePtr,vbeib.OemProductRevPtr);

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
