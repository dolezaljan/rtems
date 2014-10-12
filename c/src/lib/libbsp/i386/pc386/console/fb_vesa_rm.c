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
#include <edid.h>

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
static struct fb_var_screeninfo fb_var;
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
#define VBE_REGS_SPOT (VBE_BUF_SPOT+VBE_BUF_LEN)
/* position for real mode code reallocation to the first MB of RAM */
#define VESA_FNC_SPOT (VBE_REGS_SPOT+VBE_REG_LEN)
#define VBE_STACK_TOP (VESA_FNC_SPOT+0x500)

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

uint16_t vbe_usedMode;
void *vbe_physBasePtrOfUsedMode;
    

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
        "jmp     *%%eax\n"
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

void *rmptr_to_pmptr(void *ptr){
    uint32_t tmp = (uint32_t)ptr>>12&0xFFFF0;
    tmp += (uint32_t)ptr&0xFFFF;
    return (void *) tmp;
}

#define VBE_SIGNATURE "VESA"
#define VBE20plus_SIGNATURE "VBE2"

inline uint16_t VBEControllerInformation(struct VBE_VbeInfoBlock *infoBlock, uint16_t queriedVBEVersion) {
    struct VBE_VbeInfoBlock *VBE_buffer = (struct VBE_VbeInfoBlock *)VBE_BUF_SPOT;
    struct VBE_registers *parret = (struct VBE_registers *)VBE_REGS_SPOT;
    parret->reg_eax = VBE_RetVBEConInf;
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;
    /* indicate to graphic's bios that VBE 2.0 extended information is desired */
    if(queriedVBEVersion >= 0x200)
    {
        strncpy((char *)&VBE_buffer->VbeSignature, VBE20plus_SIGNATURE, 4*sizeof(size_t));
    }
    vbe_realmode_if();
    if((parret->reg_eax & 0xFFFF) == (VBE_callSuccessful<<8 | VBE_functionSupported))
    {
        *infoBlock = *VBE_buffer;
    }
    return (uint16_t)parret->reg_eax;
}

inline uint16_t VBEModeInformation(struct VBE_ModeInfoBlock *infoBlock, uint16_t modeNumber){
    struct VBE_ModeInfoBlock *VBE_buffer = (struct VBE_ModeInfoBlock *)VBE_BUF_SPOT;
    struct VBE_registers *parret = (struct VBE_registers *)VBE_REGS_SPOT;
    parret->reg_eax = VBE_RetVBEModInf;
    parret->reg_ecx = modeNumber;
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;
    vbe_realmode_if();
    if((parret->reg_eax & 0xFFFF) == (VBE_callSuccessful<<8 | VBE_functionSupported))
    {
        *infoBlock = *VBE_buffer;
    }
    return (uint16_t)parret->reg_eax;
}

inline uint16_t VBESetMode(uint16_t modeNumber, struct VBE_CRTCInfoBlock *infoBlock){
    struct VBE_CRTCInfoBlock *VBE_buffer = (struct VBE_CRTCInfoBlock *)VBE_BUF_SPOT;
    struct VBE_registers *parret = (struct VBE_registers *)VBE_REGS_SPOT;
    /* copy CRTC */
    *VBE_buffer = *infoBlock;
    parret->reg_eax = VBE_SetVBEMod;
    parret->reg_ebx = modeNumber;
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;
    vbe_realmode_if();
    return (uint16_t)parret->reg_eax;
}

inline uint16_t VBECurrentMode(uint16_t *modeNumber){
    struct VBE_registers *parret = (struct VBE_registers *)VBE_REGS_SPOT;
    parret->reg_eax = VBE_RetCurVBEMod;
    vbe_realmode_if();
    *modeNumber = (uint16_t)parret->reg_ebx;
    return (uint16_t)parret->reg_eax;
}

inline uint16_t VBEReportDDCCapabilities(uint16_t controllerUnitNumber, uint8_t *secondsToTransferEDIDBlock, uint8_t *DDCLevelSupported){
    struct VBE_registers *parret = (struct VBE_registers *)VBE_REGS_SPOT;
    parret->reg_eax = VBE_DisDatCha;
    parret->reg_ebx = VBEDDC_Capabilities;
    parret->reg_ecx = controllerUnitNumber;
    parret->reg_edi = 0;
    parret->reg_es = 0;
    vbe_realmode_if();
    *secondsToTransferEDIDBlock = (uint8_t)parret->reg_ebx >> 8;
    *DDCLevelSupported = (uint8_t)parret->reg_ebx;
    return (uint16_t)parret->reg_eax;
}

inline uint16_t VBEReadEDID(uint16_t controllerUnitNumber, uint16_t EDIDBlockNumber, union edid *buffer){
    union edid *VBE_buffer = (union edid *)VBE_BUF_SPOT;
    struct VBE_registers *parret = (struct VBE_registers *)VBE_REGS_SPOT;
    parret->reg_eax = VBE_DisDatCha;
    parret->reg_ebx = VBEDDC_ReadEDID;
    parret->reg_ecx = controllerUnitNumber;
    parret->reg_edx = EDIDBlockNumber;
    parret->reg_edi = (uint32_t)VBE_buffer;
    parret->reg_es = 0x0;
    vbe_realmode_if();
    if((parret->reg_eax & 0xFFFF) == (VBE_callSuccessful<<8 | VBE_functionSupported))
    {
        *buffer = *VBE_buffer;
    }
    return (uint16_t)parret->reg_eax;
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

    struct VBE_VbeInfoBlock *vib = (struct VBE_VbeInfoBlock *)VBE_BUF_SPOT;
    if(VBEControllerInformation(vib, 0x300) != (VBE_callSuccessful<<8 | VBE_functionSupported))
    {
        printk("Function 00h not supported.\n");
    }

/* see VBE CORE FUNCTIONS VERSION 3.0 Pag.65 - Appendix 1 - VBE Implementation Considerations */
#define VBE_END_OF_VideoModeList 0xFFFF
#define VBE_STUB_VideoModeList 0xFFFF
    uint16_t *modeNOPtr = (uint16_t*)rmptr_to_pmptr((void *)vib->VideoModePtr);
    uint16_t iterator = 0;
    uint16_t VideoModes[100];
    if(*(uint16_t*)vib->VideoModePtr == VBE_STUB_VideoModeList)
    {
        printk("VBE Core not implemented!\n");
    }
    else
    {
        printk("Signature: %s\n", &vib->VbeSignature);
        printk("VBE Ver  :%x\n", vib->VbeVersion);
        printk("OemString:%s\n", (char *)rmptr_to_pmptr((void *)vib->OemStringPtr));
        printk("Capabilit:0x%x\n", vib->Capabilities);
        printk("video modes: ");
        while(*(modeNOPtr+iterator) != VBE_END_OF_VideoModeList && *(modeNOPtr+iterator) != 0){ /* some bios implementations ends the list incorrectly with 0 */
            *(VideoModes+iterator) = *(modeNOPtr+iterator);
            printk("%x, ", *(VideoModes+iterator));
            iterator += sizeof(uint16_t);
        }
        *(VideoModes+iterator) = 0;
        printk("\n");
        printk("TotMemory:%d\n", vib->TotalMemory);
        printk("OemSwRev :%x\n", vib->OemSoftwareRev);
        printk("OemVenNam:%s\n", (char *)rmptr_to_pmptr((void *)vib->OemVendorNamePtr));
        printk("OemProdNm:%s\n", (char *)rmptr_to_pmptr((void *)vib->OemProductNamePtr));
        printk("OemProRev:%s\n", (char *)rmptr_to_pmptr((void *)vib->OemProductRevPtr));
    }

    BSP_wait_polled_input();

    printk("VBE/DDC capabilities:\n");
    uint8_t secTransEDIDblk, DDCsupp;
    if(VBEReportDDCCapabilities(0, &secTransEDIDblk, &DDCsupp) != (VBE_callSuccessful<<8 | VBE_functionSupported))
    {
        printk("Function 15h not supported.\n");
    }
    printk("It takes approximately %d seconds to transfer one EDID block (128 bytes).\n", secTransEDIDblk);
    printk("DDC level supported: %x\n", DDCsupp);

    BSP_wait_polled_input();
    printk("Read (E-)EDID through VBE/DDC\n");
    union edid edid;
    if(VBEReadEDID(0, 0, &edid) != (VBE_callSuccessful<<8 | VBE_functionSupported))
    {
        printk("Function 15h not supported.\n");
    }

    struct edid1 edid1;
    uint8_t checksum = 0;
    iterator = 0;
/* version of EDID structure */
    if(edid.edid1.Version == 1) { /* EDID version 1 */
        while(iterator<sizeof(struct edid1))
        {
            checksum += *((uint8_t *)&edid+iterator);
            iterator++;
        }
        if(checksum)
        {
            /* TODO: try reading EDID again */
            printk("\nchecksum failed\n");
        }
//        edid1 = &edid.edid1;
    }
    else if(edid.edid2.Version == 2) { /* EDID version 2 */
        while(iterator<sizeof(struct edid2))
        {
            checksum += *((uint8_t *)&edid+iterator);
            iterator++;
        }
        if(!checksum)
        {
            printk("EDID v2 checksum OK\n");
        }
        printk("EDID v2 not implemented\n");
    }
    else
    {
        printk("error reading EDID: no corresponding version\n");
    }
    
    BSP_wait_polled_input();

    printk("ID Manufacturer Name: %c%c%c\n",((edid1.IDManufacturerName>>10)&0x1F)+64,((edid1.IDManufacturerName>>5)&0x1F)+64,(edid1.IDManufacturerName&0x1F)+64);
    if(edid1.dtd_md[0].md.Flag0 != 0 && (edid1.dtd_md[0].md.Flag1 != 0 || edid1.dtd_md[0].md.Flag2 != 0))
    {
        printk("Optimized resolution:\n");
        printk("horizontal pixels: %d\n", edid1.dtd_md[0].dtd.HorizontalActiveHigh<<8|edid1.dtd_md[0].dtd.HorizontalActiveLow);
        printk("vertical lines: %d\n", edid1.dtd_md[0].dtd.VerticalActiveHigh<<8|edid1.dtd_md[0].dtd.VerticalActiveLow);
    }
    printk("S1: %s\n", edid1.dtd_md[2].md.DescriptorData);
    printk("S2: %s\n", edid1.dtd_md[3].md.DescriptorData);

    iterator = 0;
    struct VBE_ModeInfoBlock *mib = (struct VBE_ModeInfoBlock *)VBE_BUF_SPOT;

    while(VideoModes[iterator]!=0){
        //BSP_wait_polled_input();
        printk("Mode: %x\n", VideoModes[iterator]);
        VBEModeInformation(mib, VideoModes[iterator]);
        printk("ModeAttributes: %x\n", mib->ModeAttributes);
        uint16_t required_mode_attributes = VBE_modSupInHWMask | VBE_ColorModeMask | VBE_GraphicsModeMask | VBE_LinFraBufModeAvaiMask;
        if((mib->ModeAttributes&required_mode_attributes) == required_mode_attributes)
        {
            printk("ModeAttributes Satisfied\n");
        }else{
            printk("ModeAttributes UNSATISFIED\n");
        }
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
        //BSP_wait_polled_input();
        printk("RedMaskSize: %x\n", mib->RedMaskSize);
        printk("RedFieldPosition: %x\n", mib->RedFieldPosition);
        printk("GreenMaskSize: %x\n", mib->GreenMaskSize);
        printk("GreenFieldPosition: %x\n", mib->GreenFieldPosition);
        printk("BlueMaskSize: %x\n", mib->BlueMaskSize);
        printk("BlueFieldPosition: %x\n", mib->BlueFieldPosition);
        printk("RsvdMaskSize: %x\n", mib->RsvdMaskSize);
        printk("RsvdFieldPosition: %x\n", mib->RsvdFieldPosition);
        printk("DirectColorModeInfo: %x\n", mib->DirectColorModeInfo);
        printk("PhysBasePtr: 0x%p -- physical address for flat memory frame buffer\n", mib->PhysBasePtr);
        printk("LinBytesPerScanLine:  %x\n",  mib->LinBytesPerScanLine); 
        printk("BnkNumberOfImagePages: %x\n", mib->BnkNumberOfImagePages);
        printk("LinNumberOfImagePages: %x\n", mib->LinNumberOfImagePages);
        printk("LinRedMaskSize: %x\n",        mib->LinRedMaskSize);
        printk("LinRedFieldPosition: %x\n",   mib->LinRedFieldPosition);
        printk("LinGreenMaskSize: %x\n",      mib->LinGreenMaskSize);
        printk("LinGreenFieldPosition: %x\n", mib->LinGreenFieldPosition);
        printk("LinBlueMaskSize: %x\n",       mib->LinBlueMaskSize);
        printk("LinBlueFieldPosition: %x\n",  mib->LinBlueFieldPosition);
        printk("LinRsvdMaskSize: %x\n",       mib->LinRsvdMaskSize);
        printk("LinRsvdFieldPosition: %x\n",  mib->LinRsvdFieldPosition);
        printk("MaxPixelClock: %d\n",         mib->MaxPixelClock);
        iterator += sizeof(uint16_t);
    }

    iterator = 0;
    uint16_t optimalMode = 0, mode1024_768 = 0;
    void * optimalBasePtr = (void *)0,* pbp1024_768 =  (void *)0;

    while(VideoModes[iterator]!=0){
        VBEModeInformation(mib, VideoModes[iterator]);
        uint16_t required_mode_attributes = VBE_modSupInHWMask | VBE_ColorModeMask | VBE_GraphicsModeMask | VBE_LinFraBufModeAvaiMask;
        if((mib->ModeAttributes&required_mode_attributes) == required_mode_attributes)
        {
            if(mib->XResolution == 1024 && mib->YResolution == 768)
            {
                mode1024_768 = VideoModes[iterator];
                pbp1024_768 = mib->PhysBasePtr;
            }
            if(mib->XResolution == (edid1.dtd_md[0].dtd.HorizontalActiveHigh<<8|edid1.dtd_md[0].dtd.HorizontalActiveLow) && mib->YResolution == (edid1.dtd_md[0].dtd.VerticalActiveHigh<<8|edid1.dtd_md[0].dtd.VerticalActiveLow)){
                optimalMode = VideoModes[iterator];
                optimalBasePtr = mib->PhysBasePtr;
            }
        }
        iterator += sizeof(uint16_t);
    }
    printk("optimalMode: %x, optimBP: 0x%p\n1024x768Mode: %x, BP: 0x%p\n", optimalMode, optimalBasePtr, mode1024_768, pbp1024_768);

    /* dummy mode chooser */
    if(optimalMode!=0 && optimalBasePtr!=0){
        vbe_usedMode = optimalMode;
        vbe_physBasePtrOfUsedMode = optimalBasePtr;
    }
    else
    {
        vbe_usedMode = mode1024_768;
        vbe_physBasePtrOfUsedMode = pbp1024_768;
    }
    
    BSP_wait_polled_input();
    printk("setting mode: %x\n", vbe_usedMode);
    /* set selected mode */
    uint16_t reg_ax = VBESetMode(vbe_usedMode | VBE_linearFlatFrameBufMask,(struct VBE_CRTCInfoBlock *)(VBE_BUF_SPOT));
    if(reg_ax>>8 == VBE_callFailed)
    {
        printk("Requested mode is not available.");
    }
    if((reg_ax&0xff)!= (VBE_functionSupported | VBE_callSuccessful<<8)){
        printk("Call to function 2h failed. eax=0x%x\n", reg_ax);
    }

    /* fill framebuffer structs with info about selected mode */
    reg_ax = VBEModeInformation(mib, vbe_usedMode);
    if((reg_ax&0xff)!=VBE_functionSupported || (reg_ax>>8)!=VBE_callSuccessful){
        printk("Cannot get mode info anymore. eax=0x%x\n", reg_ax);
    }

    fb_var.xres = mib->XResolution;
    fb_var.yres = mib->YResolution;
    printk("mibXRes %d, fbxres %d, mibYRes %d, fbyres %d\n", mib->XResolution, fb_var.xres, mib->YResolution, fb_var.yres);
    fb_var.bits_per_pixel = mib->BitsPerPixel;
    printk("mibBPP %d, fbbpp %d\n", mib->BitsPerPixel, fb_var.bits_per_pixel);
    fb_var.red.offset =      mib->LinRedFieldPosition;
    fb_var.red.length =      mib->LinRedMaskSize;
    fb_var.red.msb_right =   0;
    fb_var.green.offset =    mib->LinGreenFieldPosition;
    fb_var.green.length =    mib->LinGreenMaskSize;
    fb_var.green.msb_right = 0;
    fb_var.blue.offset =     mib->LinBlueFieldPosition;
    fb_var.blue.length =     mib->LinBlueMaskSize;
    fb_var.blue.msb_right =  0;
    fb_var.transp.offset =   mib->LinRsvdFieldPosition;
    fb_var.transp.length =   mib->LinRsvdMaskSize;
    fb_var.transp.msb_right =0;

    fb_fix.smem_start  = (char *)mib->PhysBasePtr;
    fb_fix.line_length = mib->LinBytesPerScanLine;
    fb_fix.smem_len    = fb_fix.line_length*fb_var.yres;
    fb_fix.type        = FB_TYPE_PACKED_PIXELS;
    if(fb_var.bits_per_pixel < 24){
        fb_fix.visual  = FB_VISUAL_DIRECTCOLOR;
    }
    else
    {
        fb_fix.visual  = FB_VISUAL_TRUECOLOR;
    }

    printk("fb_fix and fb_var filled\n");
    BSP_wait_polled_input();

    /* try to write something to frame buffer */
    uint32_t iter = 0;
    while(iter<fb_fix.smem_len){
        *(((uint16_t *)vbe_physBasePtrOfUsedMode)+iter) = iter;
        iter++;
    }
    /* .bss section is zeroed later, backup fb_fix and fb_var */
struct fb_fix_screeninfo * pfb_fix = VESA_SPOT;
struct fb_var_screeninfo * pfb_var = VESA_SPOT+sizeof(struct fb_fix_screeninfo);
*pfb_fix = fb_fix;
*pfb_var = fb_var;
    printk("backuped\n");
    BSP_wait_polled_input();

    vib = (void *) 0;
    mib = (void *) 0;
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
    /* restore fb_fix and fb_var */
    struct fb_fix_screeninfo * pfb_fix = VESA_SPOT;
    struct fb_var_screeninfo * pfb_var = VESA_SPOT+sizeof(struct fb_fix_screeninfo);
    fb_fix = *pfb_fix;
    fb_var = *pfb_var;

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

static int get_fix_screen_info( struct fb_fix_screeninfo *info )
{
    printk("get_fix_screen_info\n");
  *info = fb_fix;
  return 0;
}

static int get_var_screen_info( struct fb_var_screeninfo *info )
{
    printk("get_var_screen_info\n");
  *info =  fb_var;
  return 0;
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
    printk("fbxres %d, fbyres %d\n", fb_var.xres, fb_var.yres);
    printk("fbbpp %d\n", fb_var.bits_per_pixel);

  switch( args->command ) {
  case FBIOGET_FSCREENINFO:
      args->ioctl_return =  get_fix_screen_info( ( struct fb_fix_screeninfo * ) args->buffer );
      break;
  case FBIOGET_VSCREENINFO:
      args->ioctl_return =  get_var_screen_info( ( struct fb_var_screeninfo * ) args->buffer );
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
