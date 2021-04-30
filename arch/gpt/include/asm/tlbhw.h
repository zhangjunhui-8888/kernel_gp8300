#ifndef _GPT_TLBHW_H_
#define _GPT_TLBHW_H_

#define BITS_MASK(width)		((1UL << (width)) - 1)

#define NUM_TLB_WAY		(1UL << DWB_WIDTH_way)
#define NUM_TLB_INDEX		(1UL << DWB_WIDTH_index)
#define NUM_TLB_ENTRY		((NUM_TLB_WAY)*(NUM_TLB_INDEX))

#define DWV_WIDTH_PROT_USER	(DWV_WIDTH_prot_user_x + DWV_WIDTH_prot_user_r + DWV_WIDTH_prot_user_w)
#define DWV_WIDTH_PROT_PRIV	(DWV_WIDTH_prot_priv_x + DWV_WIDTH_prot_priv_r + DWV_WIDTH_prot_priv_w)

#define IWV_WIDTH_PROT_USER	(IWV_WIDTH_prot_user_x + IWV_WIDTH_prot_user_r + IWV_WIDTH_prot_user_w)
#define IWV_WIDTH_PROT_PRIV	(IWV_WIDTH_prot_priv_x + IWV_WIDTH_prot_priv_r + IWV_WIDTH_prot_priv_w)

#endif /* _GPT_TLBHW_H_ */
