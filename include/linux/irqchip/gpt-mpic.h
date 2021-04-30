#ifndef _GPT_MPIC_H_
#define _GPT_MPIC_H_

#define GPT_MPIC_NR_IRQS	64
#define GPT_MPIC_MAX_IRQS       (GPT_MPIC_NR_IRQS -1)
#define GPT_MPIC_MAX_PRIORITY   (GPT_MPIC_NR_IRQS -1)

#define GPT_MPIC_OFFSET_32(base, n)            ((base) + ((n) << 2))
#define GPT_MPIC_OFFSET_32_LORH(base, n, lorh) ((base) + ((n) << 3) + ((lorh) << 2))
#define GPT_MPIC_IS_LOW_OR_HIGH(irq)           (((irq) + 32) >> 6) /* (((irq) + 32) / 64) */

/* MPINT Source Priority Register */
#define GPT_MPIC_PR_ISRC_OFF		0x0
#define GPT_MPIC_PR_ISRC(cpu)		GPT_MPIC_OFFSET_32(GPT_MPIC_PR_ISRC_OFF, (cpu))

/* MPINT Vector Address Register */
#define GPT_MPIC_VADDR_ISRC_OFF		0x100
#define GPT_MPIC_VADDR_ISRC(cpu)	GPT_MPIC_OFFSET_32(GPT_MPIC_VADDR_ISRC_OFF, (cpu))

/* MPINT Core Interrupt Enable Register */
#define GPT_MPIC_IEN_OFF		0x200
#define GPT_MPIC_IENL(cpu)		GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_IEN_OFF, (cpu), 0)
#define GPT_MPIC_IENH(cpu)		GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_IEN_OFF, (cpu), 1)

/* MPINT Core Interrupt Type Register */
/* the IRQ set (logic “0”, default) or FIQ set (logic “1”). */
#define GPT_MPIC_ITYPE_OFF		0x220
#define GPT_MPIC_ITYPEL(cpu)       GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_ITYPE_OFF, (cpu), 0)
#define GPT_MPIC_ITYPEH(cpu)       GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_ITYPE_OFF, (cpu), 1)

/* MPINT Core Vector Jump Address */
#define GPT_MPIC_VADDR_OFF		0x240
#define GPT_MPIC_VADDR(cpu)	GPT_MPIC_OFFSET_32(GPT_MPIC_VADDR_OFF, (cpu))

/* MPINT Core Masked FIQ Interrupt Register */
#define GPT_MPIC_MFIQ_OFF		0x250
#define GPT_MPIC_MFIQL(cpu)	GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_MFIQ_OFF, (cpu), 0)
#define GPT_MPIC_MFIQH(cpu)	GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_MFIQ_OFF, (cpu), 1)

/* MPINT Core Masked IRQ Interrupt Register */
#define GPT_MPIC_MIRQ_OFF		0x270
#define GPT_MPIC_MIRQL(cpu)        GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_MIRQ_OFF, (cpu), 0)
#define GPT_MPIC_MIRQH(cpu)        GPT_MPIC_OFFSET_32_LORH(GPT_MPIC_MIRQ_OFF, (cpu), 1)

/* MPINT Core Raw IRQ Interrupt Register */
#define GPT_MPIC_RAW_IRQ_OFF		0x290
#define GPT_MPIC_RAW_IRQL               GPT_MPIC_OFFSET_32(GPT_MPIC_RAW_IRQ_OFF, 0)
#define GPT_MPIC_RAW_IRQH               GPT_MPIC_OFFSET_32(GPT_MPIC_RAW_IRQ_OFF, 1)

#define GPT_MPIC_NR_CPU			4

/* MPINT Core Set SGI Regsiter */
#define GPT_MPIC_SGI_SET_OFF		0x400
#define GPT_MPIC_SGI_SET(cpu)		GPT_MPIC_OFFSET_32(GPT_MPIC_SGI_SET_OFF, (cpu))
#define GPT_MPIC_SGI_SET_OFF_ID		28
#define GPT_MPIC_SGI_SET_OFF_CPU	24

/* MPINT Core SGI Status Register */
#define GPT_MPIC_SGI_STAT_OFF		0x410
#define GPT_MPIC_SGI_STAT(cpu)		GPT_MPIC_OFFSET_32(GPT_MPIC_SGI_STAT_OFF, (cpu))

/* MPINT Core SGI Parameter Register */
#define GPT_MPIC_SGI_PARAM_OFF		0x420
#define GPT_MPIC_SGI_PARAM(cpu, sgi)	(GPT_MPIC_SGI_PARAM_OFF + (((cpu) << 3) << 2) + ((sgi) << 2))

#endif /* _GPT_MPIC_H_ */
