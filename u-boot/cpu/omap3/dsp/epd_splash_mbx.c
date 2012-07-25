
#include <common.h>

#include "dsp_cmn.h"
#include "epd_splash_mbx.h"

#if defined(SPLASH_SCREEN_DEBUG)
	#define print_info(fmt,args...)	printf ("-I-MBX-: " fmt "\n", ##args)
	#define print_warn(fmt,args...)	printf ("-W-MBX-: " fmt "\n", ##args)
#else
	#define print_info(fmt,args...)
	#define print_warn(fmt,args...)
#endif

// MAILBOX_SYSCONFIG bits
#define MAILBOX_SYSCONFIG_AUTOIDLE		(1<<0)
#define MAILBOX_SYSCONFIG_SOFTRESET		(1<<1)
enum {
	MAILBOX_SYSCONFIG_SIDLEMODE_FORCE	=	(0<<3),
	MAILBOX_SYSCONFIG_SIDLEMODE_NO		=	(1<<3),
	MAILBOX_SYSCONFIG_SIDLEMODE_SMART	=	(2<<3)
};
// MAILBOX_FIFOSTATUS_m bits
#define MAILBOX_FIFOSTATUS_m_FIFOFULLMB		(1<<0)
// MAILBOX_MSGSTATUS_m bits
#define MAILBOX_MSGSTATUS_m_NBOFMSGMB		(7<<0)
// MAILBOX_IRQENABLE bits
#define MAILBOX_IRQENABLE_NEWMSGENABLEUUMB0		(1<<0)
#define MAILBOX_IRQENABLE_NOTFULLENABLEUUMB0	(1<<1)
#define MAILBOX_IRQENABLE_NEWMSGENABLEUUMB1		(1<<2)
#define MAILBOX_IRQENABLE_NOTFULLENABLEUUMB1	(1<<3)
// MAILBOX_IRQSTATUS bits
#define MAILBOX_IRQSTATUS_NEWMSGSTATUSUUMB0		(1<<0)
#define MAILBOX_IRQSTATUS_NOTFULLSTATUSUUMB0	(1<<1)
#define MAILBOX_IRQSTATUS_NEWMSGSTATUSUUMB1		(1<<2)
#define MAILBOX_IRQSTATUS_NOTFULLSTATUSUUMB1	(1<<3)

typedef struct {
	u32	MAILBOX_REVISION;
	u32	___1[0x0003];
	u32	MAILBOX_SYSCONFIG;
	u32	MAILBOX_SYSSTATUS;
	u32	___2[0x000A];
	u32	MAILBOX_MESSAGE_m[2];
	u32	___3[0x000E];
	u32	MAILBOX_FIFOSTATUS_m[2];
	u32	___4[0x000E];
	u32	MAILBOX_MSGSTATUS_m[2];
	u32	___5[0x000E];
	struct {
		u32	MAILBOX_IRQSTATUS;
		u32	MAILBOX_IRQENABLE;
	} irq_u[2];
} MBX_REGS_t;

//#define CORE_CM_BASE	0x48004A00
#define CM_ICLKEN1_CORE		0x48004A10
#define EN_MAILBOXES_BIT	(1<<7)
//
#define MBX_REGS_BASE		0x48094000

///static u32 reg_restore_CM_ICLKEN1_CORE;

typedef struct {
	struct {
		u32	cm_iclken1_core;
	} core_cm;
} regs_restore_t;
static regs_restore_t regs_restore;

//==============================================================================
void epd_splash_mbx_init(void)
{
	volatile u32 *pCM_ICLKEN1_CORE = (u32*)CM_ICLKEN1_CORE;

	// Just enable Mailbox if not ("PRCM.CM_ICLKEN1_CORE[7] EN_MAILBOXES")
	// All other initializations are carrried out in DSP.
	regs_restore.core_cm.cm_iclken1_core = *pCM_ICLKEN1_CORE;
	*pCM_ICLKEN1_CORE |= EN_MAILBOXES_BIT;
	print_info("CM_ICLKEN1_CORE: old = 0x%.8X, new = 0x%.8X", regs_restore.core_cm.cm_iclken1_core, *pCM_ICLKEN1_CORE);
}

//==============================================================================
void epd_splash_mbx_restore(void)
{
	volatile u32 *pCM_ICLKEN1_CORE = (u32*)CM_ICLKEN1_CORE;
	u32 val;

	// Just restore the ON/OFF state. All other cleanup is carried out in DSP.
	val = (*pCM_ICLKEN1_CORE) & (~EN_MAILBOXES_BIT);
	*pCM_ICLKEN1_CORE = val | (regs_restore.core_cm.cm_iclken1_core & EN_MAILBOXES_BIT);
	print_info("CM_ICLKEN1_CORE = 0x%.8X", *pCM_ICLKEN1_CORE);
}

//==============================================================================
int epd_splash_mbx_write(const MBX_MB_en_t mbx, const u32 msg)
{
	MBX_REGS_t *pMBX = (MBX_REGS_t*)MBX_REGS_BASE;

	if (pMBX->MAILBOX_FIFOSTATUS_m[mbx] & MAILBOX_FIFOSTATUS_m_FIFOFULLMB) {
		print_warn("Message not sent: Mailbox full !!");
		return (0);	// 0 msgs sent.
	}
	pMBX->MAILBOX_MESSAGE_m[mbx] = msg;
	return (1);	// 1 msg sent.
}
