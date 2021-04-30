/*
 * OHS900 HCD (Host Controller Driver) for USB.
 *
 * Based on David Brownell's SL811 HCD
 *
 * Copyright (C) 2005 Steve Fielding
 * Copyright (C) 2004 Psion Teklogix (for NetBook PRO)
 * Copyright (C) 2004-2005 David Brownell
 * 
 * Periodic scheduling is based on Roman's OHCI code
 * 	Copyright (C) 1999 Roman Weissgaerber
 *
 * The OHS900 controller handles host side USB 
 * as well as peripheral side USB
 * This driver version doesn't implement the Gadget API
 * for the peripheral role.
 *
 * For documentation, see the OHS900 spec.
 *
 * Further changes for 2.6.22.1 kernel by Mario Becroft and Bryce Smith
 */

/*
 * Status:  Passed basic testing, works with usb-storage.
 *
 * TODO:
 * 
 * - various issues noted in the code
 * - use urb->iso_frame_desc[] with ISO transfers
 
Changelog:
//2010-09-26
//Updated to work with version 2.6.34 kernel running below 20MHz
 Tested with USB-storage, USB HID and USB-Hub. 
 Note:Issues when low speed device is connected to a HUB with high-speed connection, seems like a hardware issue. 
 Fixed: direct memory accesses replaced with ioremap_nocache.

2010-10-21
 Problem description: Unplugging the hub from the board has no effect, the de-registration seems not to work. The device is still listed in "lsusb".
 Fix: The timer functions i now queued to run from the interrupt routine after an insertion/removal event occurs. Thereby the port status is updated and the kernel
 detect the device as disconnected when checking port status.
*/

//#undef        VERBOSE
//#undef        PACKET_TRACE

/*#define DEBUG*/
#undef DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/prefetch.h>

#include <linux/ohs900.h>
#include <linux/usb/hcd.h>
#include "ohs900-hcd.h"

MODULE_DESCRIPTION("OHS900 USB Host Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ohs900-hcd");

#define DRIVER_VERSION	"19 Sep 2007"

#ifndef DEBUG
#	define	STUB_DEBUG_FILE
#endif

/* Disable the debug file since it broken at present */
//#define STUB_DEBUG_FILE

/* for now, use only one transfer register bank */
#undef	USE_B

/* this doesn't understand urb->iso_frame_desc[], but if you had a driver
 * that just queued one ISO frame per URB then iso transfers "should" work
 * using the normal urb status fields.
 */
#define	DISABLE_ISO

#define	QUIRK2
#define	QUIRK3

static const char hcd_name[] = "ohs900-hcd";

/*-------------------------------------------------------------------------*/

static irqreturn_t ohs900h_irq(struct usb_hcd *hcd);

static void port_power(struct ohs900 *s_ohs900, int is_on)
{
	struct usb_hcd *hcd = ohs900_to_hcd(s_ohs900);
	u8 tmp;

	tmp = ohs900_read(s_ohs900, OHS900_HWREVREG);

	/* hub is inactive unless the port is powered */
	if (is_on) {
		if (s_ohs900->port1 & (1 << USB_PORT_FEAT_POWER))
			return;

		s_ohs900->port1 = (1 << USB_PORT_FEAT_POWER);
		s_ohs900->irq_enable = OHS900_INTMASK_INSRMV;
	} else {
		s_ohs900->port1 = 0;
		s_ohs900->irq_enable = 0;
		hcd->state = HC_STATE_HALT;
	}

	tmp = ohs900_read(s_ohs900, OHS900_HWREVREG);

	/* hub is inactive unless the port is powered */
	if (is_on) {
		if (s_ohs900->port1 & (1 << USB_PORT_FEAT_POWER))
			return;

		s_ohs900->port1 = (1 << USB_PORT_FEAT_POWER);
		s_ohs900->irq_enable = OHS900_INTMASK_INSRMV;
	} else {
		s_ohs900->port1 = 0;
		s_ohs900->irq_enable = 0;
		hcd->state = HC_STATE_HALT;
	}

	s_ohs900->ctrl1 =
	    OHS900_TXLCTL_MASK_FS_RATE & OHS900_TXLCTL_MASK_FS_POL;

	ohs900_write(s_ohs900, OHS900_IRQ_ENABLE, 0);
	ohs900_write(s_ohs900, OHS900_IRQ_STATUS, ~0);

	if (s_ohs900->board && s_ohs900->board->port_power) {
		/* switch VBUS, at 500mA unless hub power budget gets set */
		pr_debug("power %s\n", is_on ? "on" : "off");
		s_ohs900->board->port_power(hcd->self.controller, is_on);
	}

	/* reset as thoroughly as we can */
	//if (s_ohs900->board && s_ohs900->board->reset)
	//      s_ohs900->board->reset(hcd->self.controller);
	ohs900_write(s_ohs900, OHS900_HOSTSLAVECTLREG,
		     OHS900_HSCTLREG_RESET_CORE);

	ohs900_write(s_ohs900, OHS900_IRQ_ENABLE, 0);
	ohs900_write(s_ohs900, OHS900_TXLINECTLREG, s_ohs900->ctrl1);

	ohs900_write(s_ohs900, OHS900_SOFENREG, 0);
	ohs900_write(s_ohs900, OHS900_HOSTSLAVECTLREG, OHS900_HS_CTL_INIT);

	ohs900_write(s_ohs900, OHS900_IRQ_ENABLE, s_ohs900->irq_enable);

	// if !is_on, put into lowpower mode now
}

/*-------------------------------------------------------------------------*/

/* This is a PIO-only HCD.  Queueing appends URBs to the endpoint's queue,
 * and may start I/O.  Endpoint queues are scanned during completion irq
 * handlers (one per packet: ACK, NAK, faults, etc) and urb cancelation.
 *
 * Using an external DMA engine to copy a packet at a time could work,
 * though setup/teardown costs may be too big to make it worthwhile.
 */

/* SETUP starts a new control request.  Devices are not allowed to
 * STALL or NAK these; they must cancel any pending control requests.
 */
static void setup_packet(struct ohs900 *ohs900,
			 struct ohs900h_ep *ep, struct urb *urb, u8 control)
{

	u8 addr;
	u8 len;

	ohs900_write(ohs900, OHS900_TXFIFOCONTROLREG, OHS900_FIFO_FORCE_EMPTY);
	addr = OHS900_HOST_TXFIFO_DATA;
	len = sizeof(struct usb_ctrlrequest);
	ohs900->setup_len = len;

	ohs900_write_buf(ohs900, addr, urb->setup_packet, len);
	ohs900_write(ohs900, OHS900_TXTRANSTYPEREG, OHS900_SETUP);
	ohs900_write(ohs900, OHS900_TXENDPREG, ep->epnum);
	ohs900_write(ohs900, OHS900_TXADDRREG, usb_pipedevice(urb->pipe));

	wmb();

	ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, control);
	ep->length = 0;

}

/* STATUS finishes control requests, often after IN or OUT data packets */
static void status_packet(struct ohs900 *ohs900,
			  struct ohs900h_ep *ep, struct urb *urb, u8 control)
{
	int do_out;
	do_out = urb->transfer_buffer_length && usb_pipein(urb->pipe);

	ohs900_write(ohs900, OHS900_TXTRANSTYPEREG,
		     (do_out ? OHS900_OUT_DATA1 : OHS900_IN));
	ohs900_write(ohs900, OHS900_TXENDPREG, ep->epnum);
	ohs900_write(ohs900, OHS900_TXADDRREG, usb_pipedevice(urb->pipe));
	wmb();

	ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, control);
	ep->length = 0;

}

/* IN packets can be used with any type of endpoint. here we just
 * start the transfer, data from the peripheral may arrive later.
 * urb->iso_frame_desc is currently ignored here...
 */
static void in_packet(struct ohs900 *ohs900,
		      struct ohs900h_ep *ep, struct urb *urb, u8 control)
{
	u8 len;
	len = ep->maxpacket;
	ep->length = min_t(int, len,
			   urb->transfer_buffer_length - urb->actual_length);

	ohs900_write(ohs900, OHS900_RXFIFOCONTROLREG, OHS900_FIFO_FORCE_EMPTY);
	ohs900_write(ohs900, OHS900_TXTRANSTYPEREG, OHS900_IN);
	ohs900_write(ohs900, OHS900_TXADDRREG, usb_pipedevice(urb->pipe));
	ohs900_write(ohs900, OHS900_TXENDPREG, ep->epnum);
	ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, control);

}

/* OUT packets can be used with any type of endpoint.
 * urb->iso_frame_desc is currently ignored here...
 */
static void out_packet(struct ohs900 *ohs900,
		       struct ohs900h_ep *ep, struct urb *urb, u8 control)
{
	void *buf;
	u8 len;

	buf = urb->transfer_buffer + urb->actual_length;
	prefetch(buf);

	len = min_t(int, ep->maxpacket,
		    urb->transfer_buffer_length - urb->actual_length);

	ohs900_write(ohs900, OHS900_TXFIFOCONTROLREG, OHS900_FIFO_FORCE_EMPTY);
	if (!(control & OHS900_HCTLMASK_ISO_EN)
	    && usb_gettoggle(urb->dev, ep->epnum, 1))
		ohs900_write(ohs900, OHS900_TXTRANSTYPEREG, OHS900_OUT_DATA1);
	else
		ohs900_write(ohs900, OHS900_TXTRANSTYPEREG, OHS900_OUT_DATA0);

	ohs900_write_buf(ohs900, OHS900_HOST_TXFIFO_DATA, buf, len);

	ohs900_write(ohs900, OHS900_TXADDRREG, usb_pipedevice(urb->pipe));
	ohs900_write(ohs900, OHS900_TXENDPREG, ep->epnum);
	ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, control);

	ep->length = len;

}

/*-------------------------------------------------------------------------*/

/* caller updates on-chip enables later */

static inline void sofirq_on(struct ohs900 *ohs900)
{
	if (ohs900->irq_enable & OHS900_INTMASK_SOFINTR)
		return;
	pr_debug("sof irq on\n");
	ohs900->irq_enable |= OHS900_INTMASK_SOFINTR;
}

static inline void sofirq_off(struct ohs900 *ohs900)
{
	if (!(ohs900->irq_enable & OHS900_INTMASK_SOFINTR))
		return;
	pr_debug("sof irq off\n");
	ohs900->irq_enable &= ~OHS900_INTMASK_SOFINTR;
}

/*-------------------------------------------------------------------------*/

/* pick the next endpoint for a transaction, and issue it.
 * frames start with periodic transfers (after whatever is pending
 * from the previous frame), and the rest of the time is async
 * transfers, scheduled round-robin.
 */
static struct ohs900h_ep *start(struct ohs900 *ohs900)
{
	struct ohs900h_ep *ep;
	struct urb *urb;
	int fclock;
	u8 control;

	/* use endpoint at schedule head */
	if (ohs900->next_periodic) {
		ep = ohs900->next_periodic;
		ohs900->next_periodic = ep->next;
	} else {
		if (ohs900->next_async)
			ep = ohs900->next_async;
		else if (!list_empty(&ohs900->async))
			ep = container_of(ohs900->async.next,
					  struct ohs900h_ep, schedule);
		else {
			/* could set up the first fullspeed periodic
			 * transfer for the next frame ...
			 */
			return NULL;
		}

		if (ep->schedule.next == &ohs900->async)
			ohs900->next_async = NULL;
		else
			ohs900->next_async = container_of(ep->schedule.next,
							  struct ohs900h_ep,
							  schedule);
	}

	if (unlikely(list_empty(&ep->hep->urb_list))) {
		pr_debug("empty %p queue?\n", ep);
		return NULL;
	}

	urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
	control = ep->defctrl;

	/* if this frame doesn't have enough time left to transfer this
	 * packet, wait till the next frame.  too-simple algorithm...
	 */
	fclock = 12000 - (ohs900_read(ohs900, OHS900_SOFTMRREG) << 6);
	fclock -= 100;		/* setup takes not much time */
	fclock -= 1500;		/* Margin to workaround too-long-frame bug */
	if (urb->dev->speed == USB_SPEED_LOW) {
		if (control & OHS900_HCTLMASK_PREAMBLE_EN) {
			/* also note erratum 1: some hubs won't work */
			fclock -= 800;
		}
		fclock -= ep->maxpacket << 8;

		/* erratum 2: AFTERSOF only works for fullspeed */
		if (fclock < 0) {
			if (ep->period)
				ohs900->stat_overrun++;
			sofirq_on(ohs900);
			return NULL;
		}
	} else {
		fclock -= 12000 / 19;	/* 19 64byte packets/msec */
		if (fclock < 0) {
			if (ep->period)
				ohs900->stat_overrun++;
			control |= OHS900_HCTLMASK_SOF_SYNC;

			/* throttle bulk/control irq noise */
		} else if (ep->nak_count)
			control |= OHS900_HCTLMASK_SOF_SYNC;
	}

	switch (ep->nextpid) {
	case USB_PID_IN:
		in_packet(ohs900, ep, urb, control);
		break;
	case USB_PID_OUT:
		out_packet(ohs900, ep, urb, control);
		break;
	case USB_PID_SETUP:
		setup_packet(ohs900, ep, urb, control);
		break;
	case USB_PID_ACK:	/* for control status */
		status_packet(ohs900, ep, urb, control);
		break;
	default:
		pr_debug("bad ep%p pid %02x\n", ep, ep->nextpid);
		ep = NULL;
	}
	return ep;
}

#define MIN_JIFFIES	((msecs_to_jiffies(2) > 1) ? msecs_to_jiffies(2) : 2)

static inline void start_transfer(struct ohs900 *ohs900)
{
	if (ohs900->port1 & (1 << USB_PORT_FEAT_SUSPEND))
		return;
	if (ohs900->active_a == NULL) {
		ohs900->active_a = start(ohs900);
		if (ohs900->active_a != NULL)
			ohs900->jiffies_a = jiffies + MIN_JIFFIES;
	}
}

static void finish_request(struct ohs900 *ohs900,
			   struct ohs900h_ep *ep,
			   struct urb *urb, int status) __releases(ohs900->lock)
{
	unsigned i;

	urb->hcpriv = NULL;

	if (usb_pipecontrol(urb->pipe))
		ep->nextpid = USB_PID_SETUP;

	usb_hcd_unlink_urb_from_ep(ohs900_to_hcd(ohs900), urb);
	spin_lock(&ohs900->lock);
	usb_hcd_giveback_urb(ohs900_to_hcd(ohs900), urb, status);
	spin_unlock(&ohs900->lock);

	/* leave active endpoints in the schedule */
	if (!list_empty(&ep->hep->urb_list)) {

		return;
	}
	/* async deschedule? */
	if (!list_empty(&ep->schedule)) {

		list_del_init(&ep->schedule);
		if (ep == ohs900->next_async)
			ohs900->next_async = NULL;

		return;
	}

	/* periodic deschedule */
	for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period) {
		struct ohs900h_ep *temp;
		struct ohs900h_ep **prev = &ohs900->periodic[i];

		while (*prev && ((temp = *prev) != ep))
			prev = &temp->next;
		if (*prev)
			*prev = ep->next;
		ohs900->load[i] -= ep->load;
	}

	ep->branch = PERIODIC_SIZE;
	ohs900->periodic_count--;
	ohs900_to_hcd(ohs900)->self.bandwidth_allocated
	    -= ep->load / ep->period;
	if (ep == ohs900->next_periodic)
		ohs900->next_periodic = ep->next;

	/* we might turn SOFs back on again for the async schedule */
	if (ohs900->periodic_count == 0)
		sofirq_off(ohs900);

}

static void done(struct ohs900 *ohs900, struct ohs900h_ep *ep)
{
	u8 status;
	struct urb *urb;
	int urbstat = -EINPROGRESS;

	if (unlikely(!ep))
		return;

	status = ohs900_read(ohs900, OHS900_HRXSTATREG);

	ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, 0);
	urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);

	/* we can safely ignore NAKs */
	if (status & OHS900_STATMASK_NAK_RXED) {

		if (!ep->period)
			ep->nak_count++;
		ep->error_count = 0;

		/* ACK, or IN with no errors, advances transfer, toggle, and maybe queue */
	} else if (status & OHS900_STATMASK_ACK_RXED
		   || ((status & ~OHS900_STATMASK_DATA_SEQ) == 0)) {
		struct usb_device *udev = urb->dev;
		int len;
		unsigned char *buf;

		/* urb->iso_frame_desc is currently ignored here... */

		ep->nak_count = ep->error_count = 0;
		switch (ep->nextpid) {
		case USB_PID_OUT:

			urb->actual_length += ep->length;
			usb_dotoggle(udev, ep->epnum, 1);
			if (urb->actual_length == urb->transfer_buffer_length) {
				if (usb_pipecontrol(urb->pipe))
					ep->nextpid = USB_PID_ACK;

				/* some bulk protocols terminate OUT transfers
				 * by a short packet, using ZLPs not padding.
				 */
				else if (ep->length < ep->maxpacket
					 || !(urb->transfer_flags
					      & URB_ZERO_PACKET))
					urbstat = 0;
			}
			break;
		case USB_PID_IN:
			//pr_debug("ACK/in qh%p\n", ep);
			buf = urb->transfer_buffer + urb->actual_length;
			//prefetchw(buf);
			len = ohs900_read(ohs900, OHS900_RXFIFOCNTLSBREG)
			    + (ohs900_read(ohs900,
					   OHS900_RXFIFOCNTMSBREG) << 8);

			if (len > ep->length) {
				len = ep->length;
				urbstat = -EOVERFLOW;
				pr_debug("EOVERFLOW\n");
			}
			urb->actual_length += len;
			ohs900_read_buf(ohs900, OHS900_HOST_RXFIFO_DATA,
					buf, len, urb->actual_length);

			if (urb->actual_length > urb->transfer_buffer_length)
				pr_debug("Actual len %d > buff len %d, ERROR\n",
				    urb->actual_length,
				    urb->transfer_buffer_length);
			usb_dotoggle(udev, ep->epnum, 0);
			if (urbstat == -EINPROGRESS
			    && (len < ep->maxpacket
				|| urb->actual_length ==
				urb->transfer_buffer_length)) {

				if (usb_pipecontrol(urb->pipe))
					ep->nextpid = USB_PID_ACK;
				else
					urbstat = 0;
			}

			break;

		case USB_PID_SETUP:
			if (urb->transfer_buffer_length == urb->actual_length)
				ep->nextpid = USB_PID_ACK;
			else if (usb_pipeout(urb->pipe)) {
				usb_settoggle(udev, 0, 1, 1);
				ep->nextpid = USB_PID_OUT;
			} else {
				usb_settoggle(udev, 0, 0, 1);
				ep->nextpid = USB_PID_IN;
			}
			break;
		case USB_PID_ACK:
			ohs900->setup_stage = 0;
			urbstat = 0;
			break;
		}

		/* STALL stops all transfers */
	} else if (status & OHS900_STATMASK_STALL_RXED) {
		PACKET("...STALL qh%p\n", ep);
		ep->nak_count = ep->error_count = 0;
		urbstat = -EPIPE;

		/* error? retry, until "3 strikes" */
	} else if (++ep->error_count >= 5) {
		if (status & OHS900_STATMASK_RX_TMOUT)
			urbstat = -ETIME;
		else if (status & OHS900_STATMASK_RX_OVF)
			urbstat = -EOVERFLOW;
		else
			urbstat = -EPROTO;
		ep->error_count = 0;
		pr_debug("...5STRIKES %02x qh%p stat %d\n", status, ep, urbstat);
	}

	if (urbstat != -EINPROGRESS || urb->unlinked) {
		finish_request(ohs900, ep, urb, urbstat);
	}
}

static inline u8 checkdone(struct ohs900 *ohs900)
{
	u8 ctl;
	u8 irqstat = 0;

	if (ohs900->active_a && time_before_eq(ohs900->jiffies_a, jiffies)) {
		ctl = ohs900_read(ohs900, OHS900_HOST_TX_CTLREG);
		if (ctl & OHS900_HCTLMASK_TRANS_REQ)
			ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, 0);
		pr_debug("%s DONE_A: ctrl %02x sts %02x\n",
		    (ctl & OHS900_HCTLMASK_TRANS_REQ) ? "timeout" : "lost",
		    ctl, ohs900_read(ohs900, OHS900_HRXSTATREG));
		irqstat |= OHS900_INTMASK_TRANS_DONE;
	}
	return irqstat;
}

static irqreturn_t ohs900h_irq(struct usb_hcd *hcd)
{
	struct ohs900 *ohs900 = hcd_to_ohs900(hcd);
	u8 irqstat;
	irqreturn_t ret = IRQ_NONE;
	unsigned retries = 5;

	spin_lock(&ohs900->lock);

retry:
	irqstat = ohs900_read(ohs900, OHS900_IRQ_STATUS);

	if (irqstat) {
		ohs900_write(ohs900, OHS900_IRQ_STATUS, irqstat);
		irqstat &= ohs900->irq_enable;
	}
#ifdef	QUIRK2
	/* this may no longer be necessary ... */
	if (irqstat == 0) {
		irqstat = checkdone(ohs900);
		if (irqstat)
			ohs900->stat_lost++;
	}
#endif

	/* USB packets, not necessarily handled in the order they're
	 * issued ... that's fine if they're different endpoints.
	 */
	if (irqstat & OHS900_INTMASK_TRANS_DONE) {
		done(ohs900, ohs900->active_a);
		ohs900->active_a = NULL;
		ohs900->stat_a++;
	}

	if (irqstat & OHS900_INTMASK_SOFINTR) {
		unsigned index;

		index = ohs900->frame++ % (PERIODIC_SIZE - 1);
		ohs900->stat_sof++;

		/* be graceful about almost-inevitable periodic schedule
		 * overruns:  continue the previous frame's transfers iff
		 * this one has nothing scheduled.
		 */
		if (ohs900->next_periodic) {
			// ERR("overrun to slot %d\n", index);
			ohs900->stat_overrun++;
		}
		if (ohs900->periodic[index])
			ohs900->next_periodic = ohs900->periodic[index];
	}

	/* khubd manages debouncing and wakeup */
	if (irqstat & OHS900_INTMASK_INSRMV) {
		ohs900->stat_insrmv++;

		/* most stats are reset for each VBUS session */
		ohs900->stat_wake = 0;
		ohs900->stat_sof = 0;
		ohs900->stat_a = 0;
		ohs900->stat_b = 0;
		ohs900->stat_lost = 0;

		ohs900->ctrl1 = 0;
		ohs900_write(ohs900, OHS900_TXLINECTLREG, ohs900->ctrl1);

		ohs900->irq_enable = OHS900_INTMASK_INSRMV;
		ohs900_write(ohs900, OHS900_IRQ_ENABLE, ohs900->irq_enable);

		/* usbcore nukes other pending transactions on disconnect */
		if (ohs900->active_a) {
			ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, 0);
			finish_request(ohs900, ohs900->active_a,
				       container_of(ohs900->active_a->hep->
						    urb_list.next, struct urb,
						    urb_list), -ESHUTDOWN);
			ohs900->active_a = NULL;
		}

		/* port status seems wierd until after reset, so
		 * force the reset and make khubd clean up later.
		 */
		ohs900->port1 |= (1 << USB_PORT_FEAT_C_CONNECTION)
		    | (1 << USB_PORT_FEAT_CONNECTION);

		mod_timer(&ohs900->timer, jiffies + msecs_to_jiffies(1800));

	} else if (irqstat & OHS900_INTMASK_RESUME_DET) {
		if (ohs900->port1 & (1 << USB_PORT_FEAT_SUSPEND)) {
			pr_debug("wakeup\n");
			ohs900->port1 |= 1 << USB_PORT_FEAT_C_SUSPEND;
			ohs900->stat_wake++;
		} else
			irqstat &= ~OHS900_INTMASK_RESUME_DET;
	}

	if (irqstat) {
		if (ohs900->port1 & (1 << USB_PORT_FEAT_ENABLE))
			start_transfer(ohs900);
		ret = IRQ_HANDLED;
		if (retries--)
			goto retry;
	}

	if (ohs900->periodic_count == 0 && list_empty(&ohs900->async))
		sofirq_off(ohs900);
	ohs900_write(ohs900, OHS900_IRQ_ENABLE, ohs900->irq_enable);

	spin_unlock(&ohs900->lock);

	return ret;
}

/*-------------------------------------------------------------------------*/

/* usb 1.1 says max 90% of a frame is available for periodic transfers.
 * this driver doesn't promise that much since it's got to handle an
 * IRQ per packet; irq handling latencies also use up that time.
 */
#define	MAX_PERIODIC_LOAD	500	/* out of 1000 usec */

static int balance(struct ohs900 *ohs900, u16 period, u16 load)
{
	int i, branch = -ENOSPC;
	/* search for the least loaded schedule branch of that period
	 * which has enough bandwidth left unreserved.
	 */
	for (i = 0; i < period; i++) {
		if (branch < 0 || ohs900->load[branch] > ohs900->load[i]) {
			int j;

			for (j = i; j < PERIODIC_SIZE; j += period) {
				if ((ohs900->load[j] + load)
				    > MAX_PERIODIC_LOAD)
					break;
			}
			if (j < PERIODIC_SIZE)
				continue;
			branch = i;
		}
	}
	return branch;
}

/*-------------------------------------------------------------------------*/

static int ohs900h_urb_enqueue(struct usb_hcd *hcd,
			       struct urb *urb, gfp_t mem_flags)
{
	struct ohs900 *ohs900a = hcd_to_ohs900(hcd);
	struct usb_device *udev = urb->dev;
	unsigned int pipe = urb->pipe;
	int is_out = !usb_pipein(pipe);
	int type = usb_pipetype(pipe);
	int epnum = usb_pipeendpoint(pipe);
	struct ohs900h_ep *ep = NULL;
	unsigned long flags;
	int i;
	int retval;
	struct usb_host_endpoint *hep = urb->ep;

#ifdef	DISABLE_ISO
	if (type == PIPE_ISOCHRONOUS)
		return -ENOSPC;
#endif

	/* avoid all allocations within spinlocks */
	if (!hep->hcpriv)
		ep = kzalloc(sizeof *ep, mem_flags);

	spin_lock_irqsave(&ohs900a->lock, flags);

	/* don't submit to a dead or disabled port */
	if (!(ohs900a->port1 & (1 << USB_PORT_FEAT_ENABLE))
	    || !HC_IS_RUNNING(hcd->state)) {
		retval = -ENODEV;
		kfree(ep);
		goto fail_not_linked;
	}
	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	if (retval) {
		kfree(ep);
		goto fail_not_linked;
	}

	if (hep->hcpriv) {
		ep = hep->hcpriv;
	} else if (!ep) {
		retval = -ENOMEM;
		goto fail;

	} else {
		INIT_LIST_HEAD(&ep->schedule);
		ep->udev = udev;
		ep->epnum = epnum;
		ep->maxpacket = usb_maxpacket(udev, urb->pipe, is_out);
		ep->defctrl = OHS900_HCTLMASK_TRANS_REQ;
		usb_settoggle(udev, epnum, is_out, 0);

		if (type == PIPE_CONTROL)
			ep->nextpid = USB_PID_SETUP;
		else if (is_out)
			ep->nextpid = USB_PID_OUT;
		else
			ep->nextpid = USB_PID_IN;

		if (ep->maxpacket > H_MAXPACKET) {
			/* iso packets up to 64 bytes could work... */
			pr_debug("dev %d ep%d maxpacket %d\n",
			     udev->devnum, epnum, ep->maxpacket);
			retval = -EINVAL;
			goto fail;
		}

		if (udev->speed == USB_SPEED_LOW) {
			/* send preamble for external hub? */
			if (ohs900a->ctrl1 & OHS900_TXLCTL_MASK_FS_RATE)
				ep->defctrl |= OHS900_HCTLMASK_PREAMBLE_EN;
		}
		switch (type) {
		case PIPE_ISOCHRONOUS:
		case PIPE_INTERRUPT:
			if (urb->interval > PERIODIC_SIZE)
				urb->interval = PERIODIC_SIZE;
			ep->period = urb->interval;
			ep->branch = PERIODIC_SIZE;
			if (type == PIPE_ISOCHRONOUS)
				ep->defctrl |= OHS900_HCTLMASK_ISO_EN;
			ep->load = usb_calc_bus_time(udev->speed, !is_out,
						     (type == PIPE_ISOCHRONOUS),
						     usb_maxpacket(udev, pipe,
								   is_out))
			    / 1000;
			break;
		}

		hep->hcpriv = ep;
		ep->hep = hep;
	}

	/* maybe put endpoint into schedule */
	switch (type) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		if (list_empty(&ep->schedule))
			list_add_tail(&ep->schedule, &ohs900a->async);
		break;
	case PIPE_ISOCHRONOUS:
	case PIPE_INTERRUPT:
		urb->interval = ep->period;
		if (ep->branch < PERIODIC_SIZE)
			break;

		retval = balance(ohs900a, ep->period, ep->load);
		if (retval < 0)
			goto fail;
		ep->branch = retval;
		retval = 0;
		urb->start_frame = (ohs900a->frame & (PERIODIC_SIZE - 1))
		    + ep->branch;

		/* sort each schedule branch by period (slow before fast)
		 * to share the faster parts of the tree without needing
		 * dummy/placeholder nodes
		 */
		pr_debug("schedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
		for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period) {
			struct ohs900h_ep **prev = &ohs900a->periodic[i];
			struct ohs900h_ep *here = *prev;

			while (here && ep != here) {
				if (ep->period > here->period)
					break;
				prev = &here->next;
				here = *prev;
			}
			if (ep != here) {
				ep->next = here;
				*prev = ep;
			}
			ohs900a->load[i] += ep->load;
		}
		ohs900a->periodic_count++;
		hcd->self.bandwidth_allocated += ep->load / ep->period;
		sofirq_on(ohs900a);
	}

	urb->hcpriv = hep;

	start_transfer(ohs900a);
	//printk("IRQ ENA:: %d\n", ohs900a->irq_enable);
	ohs900_write(ohs900a, OHS900_IRQ_ENABLE, ohs900a->irq_enable);
	if (retval)
		usb_hcd_unlink_urb_from_ep(hcd, urb);
	spin_unlock_irqrestore(&ohs900a->lock, flags);
	return retval;

fail:	pr_debug("FAIL URB ENQUE");
	if (retval)
		usb_hcd_unlink_urb_from_ep(hcd, urb);
fail_not_linked:pr_debug("FAIL URB ENQUE - not_linked");
	spin_unlock_irqrestore(&ohs900a->lock, flags);
	return retval;
}

static int ohs900h_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct ohs900 *ohs900 = hcd_to_ohs900(hcd);
	struct usb_host_endpoint *hep = urb->hcpriv;
	unsigned long flags;
	struct ohs900h_ep *ep;
	int retval;

	if (!hep)
		return -EINVAL;

	spin_lock_irqsave(&ohs900->lock, flags);
	retval = usb_hcd_check_unlink_urb(hcd, urb, status);
	ep = hep->hcpriv;
	if (retval)
		goto fail;

	hep = hep->hcpriv;
	if (ep) {
		/* finish right away if this urb can't be active ...
		 * note that some drivers wrongly expect delays
		 */
		if (ep->hep->urb_list.next != &urb->urb_list) {
			/* not front of queue?  never active */

			/* for active transfers, we expect an IRQ */
		} else if (ohs900->active_a == ep) {
			if (time_before_eq(ohs900->jiffies_a, jiffies)) {
				/* happens a lot with lowspeed?? */
				pr_debug("giveup on DONE_A: ctrl %02x sts %02x\n",
				    ohs900_read(ohs900, OHS900_HOST_TX_CTLREG),
				    ohs900_read(ohs900, OHS900_HRXSTATREG));
				ohs900_write(ohs900, OHS900_HOST_TX_CTLREG, 0);
				ohs900->active_a = NULL;
			} else
				urb = NULL;

		} else {
			/* front of queue for inactive endpoint */
		}

		if (urb)
			finish_request(ohs900, ep, urb, 0);
		else
			pr_debug("dequeue, urb %p active %s; wait4irq\n", urb,
			     (ohs900->active_a == ep) ? "A" : "B");
	}

fail:
	spin_unlock_irqrestore(&ohs900->lock, flags);
	return retval;
}

static void
ohs900h_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct ohs900h_ep *ep = hep->hcpriv;

	if (!ep)
		return;

	/* assume we'd just wait for the irq */
	if (!list_empty(&hep->urb_list))
		msleep(3);
	if (!list_empty(&hep->urb_list))
		WARNING("ep %p not empty?\n", ep);

	kfree(ep);
	hep->hcpriv = NULL;
}

static int ohs900h_get_frame(struct usb_hcd *hcd)
{
	struct ohs900 *ohs900 = hcd_to_ohs900(hcd);

	/* wrong except while periodic transfers are scheduled;
	 * never matches the on-the-wire frame;
	 * subject to overruns.
	 */
	return ohs900->frame;
}

/*-------------------------------------------------------------------------*/

/* the virtual root hub timer IRQ checks for hub status */
static int ohs900h_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct ohs900 *ohs900 = hcd_to_ohs900(hcd);
#ifdef	QUIRK3
	unsigned long flags;
	//pr_debug("Enters ohs900h_hub_status_data  xxx");
	/* non-SMP HACK: use root hub timer as i/o watchdog
	 * this seems essential when SOF IRQs aren't in use...
	 */
	local_irq_save(flags);
	if (!timer_pending(&ohs900->timer)) {
		if (ohs900h_irq( /* ~0, */ hcd) != IRQ_NONE)
			ohs900->stat_lost++;
	}
	local_irq_restore(flags);
#endif

	if (!(ohs900->port1 & (0xffff << 16)))
		return 0;

	/* tell khubd port 1 changed */
	*buf = (1 << 1);
	return 1;
}

static void
ohs900h_hub_descriptor(struct ohs900 *ohs900, struct usb_hub_descriptor *desc)
{
	u16 temp;

	desc->bDescriptorType = 0x29;
	desc->bHubContrCurrent = 0;

	desc->bNbrPorts = 1;
	desc->bDescLength = 7 + 2 * desc->bNbrPorts;

	temp = 1 + (desc->bNbrPorts / 8);
	/* two bitmaps:  ports removable, and usb 1.0 legacy PortPwrCtrlMask */
	memset(&desc->u.hs.DeviceRemovable[0], 0, temp);
	memset(&desc->u.hs.DeviceRemovable[temp], 0xff, temp);

	/* per-port power switching (gang of one!), or none */
	desc->bPwrOn2PwrGood = 0;
	if (ohs900->board && ohs900->board->port_power) {
		desc->bPwrOn2PwrGood = ohs900->board->potpg;
		if (!desc->bPwrOn2PwrGood)
			desc->bPwrOn2PwrGood = 10;
		temp = 0x0001;	/* per-port power control */
	} else
		temp = 0x0002;	/* no power switching */

	/* no overcurrent errors detection/handling */
	temp |= 0x0010;

	desc->wHubCharacteristics = cpu_to_le16(temp);
}

static void ohs900h_timer(unsigned long _ohs900)
{
	struct ohs900 *ohs900 = (void *)_ohs900;
	unsigned long flags;
	u8 irqstat;
	u8 signaling = ohs900->ctrl1 & OHS900_TXLCTL_MASK_LINE_CTRL_BITS;
	const u32 mask = (1 << USB_PORT_FEAT_CONNECTION)
	    | (1 << USB_PORT_FEAT_ENABLE)
	    | (1 << USB_PORT_FEAT_LOWSPEED);
	u8 sofEnReg = 0;

	spin_lock_irqsave(&ohs900->lock, flags);

	/* stop special signaling */
	ohs900->ctrl1 &= ~OHS900_TXLCTL_MASK_FORCE;
	ohs900_write(ohs900, OHS900_TXLINECTLREG, ohs900->ctrl1);
	udelay(3);

	irqstat = ohs900_read(ohs900, OHS900_IRQ_STATUS);

	switch (signaling) {
	case OHS900_TXLCTL_MASK_SE0:
		pr_debug("end reset\n");
		ohs900->port1 = (1 << USB_PORT_FEAT_C_RESET)
		    | (1 << USB_PORT_FEAT_POWER);
		ohs900->ctrl1 = 0;
		/* don't wrongly ack RD */
		if (irqstat & OHS900_INTMASK_INSRMV)
			irqstat &= ~OHS900_INTMASK_RESUME_DET;
		break;
	case OHS900_TXLCTL_MASK_FS_K:
		pr_debug("end resume\n");
		ohs900->port1 &= ~(1 << USB_PORT_FEAT_SUSPEND);
		break;
	default:
		pr_debug("odd timer signaling: %02x\n", signaling);
		break;
	}
	ohs900_write(ohs900, OHS900_IRQ_STATUS, irqstat);

	//if (irqstat & OHS900_INTMASK_RESUME_DET) {...
	if (ohs900_read(ohs900, OHS900_RXCONNSTATEREG) ==
	    OHS900_DISCONNECT_STATE) {
		/* usbcore nukes all pending transactions on disconnect */
		if (ohs900->port1 & (1 << USB_PORT_FEAT_CONNECTION))
			ohs900->port1 |= (1 << USB_PORT_FEAT_C_CONNECTION)
			    | (1 << USB_PORT_FEAT_C_ENABLE);
		ohs900->port1 &= ~mask;
		ohs900->irq_enable = OHS900_INTMASK_INSRMV;
	} else {
		ohs900->port1 |= mask;
		if (ohs900_read(ohs900, OHS900_RXCONNSTATEREG) &
		    OHS900_FS_CONN_STATE)
			ohs900->port1 &= ~(1 << USB_PORT_FEAT_LOWSPEED);
		ohs900->irq_enable =
		    OHS900_INTMASK_INSRMV | OHS900_INTMASK_RESUME_DET;
	}
	//...presumably ?...}
	if (ohs900->port1 & (1 << USB_PORT_FEAT_CONNECTION)) {

		ohs900->irq_enable |= OHS900_INTMASK_TRANS_DONE;
		if (ohs900->port1 & (1 << USB_PORT_FEAT_LOWSPEED)) {
			ohs900->ctrl1 &= ~OHS900_TXLCTL_MASK_FS_POL;
			ohs900->ctrl1 &= ~OHS900_TXLCTL_MASK_FS_RATE;
		} else {
			ohs900->ctrl1 |= OHS900_TXLCTL_MASK_FS_POL;
			ohs900->ctrl1 |= OHS900_TXLCTL_MASK_FS_RATE;
		}

		/* start SOFs flowing, kickstarting with A registers */
		sofEnReg = OHS900_MASK_SOF_ENA;

		/* Set default device address */
		ohs900_write(ohs900, OHS900_TXADDRREG, 0);

		/* khubd provides debounce delay */
	} else {
		ohs900->ctrl1 = 0;
	}
	ohs900_write(ohs900, OHS900_TXLINECTLREG, ohs900->ctrl1);
	ohs900_write(ohs900, OHS900_SOFENREG, sofEnReg);

	/* reenable irqs */

	ohs900_write(ohs900, OHS900_IRQ_ENABLE, ohs900->irq_enable);
	spin_unlock_irqrestore(&ohs900->lock, flags);
}

static int
ohs900h_hub_control(struct usb_hcd *hcd,
		    u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct ohs900 *ohs900 = hcd_to_ohs900(hcd);
	int retval = 0;
	unsigned long flags;

	spin_lock_irqsave(&ohs900->lock, flags);

	switch (typeReq) {
	case ClearHubFeature:
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case ClearPortFeature:
		if (wIndex != 1 || wLength != 0)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			ohs900->port1 &= (1 << USB_PORT_FEAT_POWER);
			ohs900->ctrl1 = 0;
			ohs900_write(ohs900, OHS900_SOFENREG, 0);
			ohs900_write(ohs900, OHS900_TXLINECTLREG,
				     ohs900->ctrl1);
			ohs900->irq_enable = OHS900_INTMASK_INSRMV;
			ohs900_write(ohs900, OHS900_IRQ_ENABLE,
				     ohs900->irq_enable);
			break;
		case USB_PORT_FEAT_SUSPEND:
			if (!(ohs900->port1 & (1 << USB_PORT_FEAT_SUSPEND)))
				break;

			/* 20 msec of resume/K signaling, other irqs blocked */
			pr_debug("start resume...\n");
			ohs900->irq_enable = 0;
			ohs900_write(ohs900, OHS900_IRQ_ENABLE,
				     ohs900->irq_enable);
			ohs900->ctrl1 |= OHS900_TXLCTL_MASK_FS_K;
			ohs900_write(ohs900, OHS900_TXLINECTLREG,
				     ohs900->ctrl1);

			mod_timer(&ohs900->timer, jiffies
				  + msecs_to_jiffies(20));
			break;
		case USB_PORT_FEAT_POWER:
			port_power(ohs900, 0);
			break;
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
			break;
		default:
			goto error;
		}
		ohs900->port1 &= ~(1 << wValue);
		break;
	case GetHubDescriptor:
		ohs900h_hub_descriptor(ohs900,
				       (struct usb_hub_descriptor *)buf);
		break;
	case GetHubStatus:
		*(__le32 *) buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			goto error;
		*(__le32 *) buf = cpu_to_le32(ohs900->port1);

#ifndef	VERBOSE
		if (*(u16 *) (buf + 2))	/* only if wPortChange is interesting */
#endif

#ifdef CONFIG_USB_SUSPEND
			pr_debug("\n USB_SUSPEND ENABLED!!!");

#endif

		pr_debug("GetPortStatus %08x\n", ohs900->port1);
		break;
	case SetPortFeature:
		if (wIndex != 1 || wLength != 0)
			goto error;
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			if (ohs900->port1 & (1 << USB_PORT_FEAT_RESET))
				goto error;
			if (!(ohs900->port1 & (1 << USB_PORT_FEAT_ENABLE)))
				goto error;

			pr_debug("suspend...\n");
			ohs900_write(ohs900, OHS900_SOFENREG, 0);
			break;
		case USB_PORT_FEAT_POWER:
			port_power(ohs900, 1);
			break;
		case USB_PORT_FEAT_RESET:
			if (ohs900->port1 & (1 << USB_PORT_FEAT_SUSPEND))
				goto error;
			if (!(ohs900->port1 & (1 << USB_PORT_FEAT_POWER)))
				break;

			/* 50 msec of reset/SE0 signaling, irqs blocked */
			ohs900->irq_enable = 0;
			ohs900_write(ohs900, OHS900_IRQ_ENABLE,
				     ohs900->irq_enable);
			ohs900_write(ohs900, OHS900_SOFENREG, 0);
			ohs900->ctrl1 = OHS900_TXLCTL_MASK_SE0;
			ohs900_write(ohs900, OHS900_TXLINECTLREG,
				     ohs900->ctrl1);
			ohs900->port1 |= (1 << USB_PORT_FEAT_RESET);
			mod_timer(&ohs900->timer, jiffies
				  + msecs_to_jiffies(50));
			break;
		default:
			goto error;
		}
		ohs900->port1 |= 1 << wValue;
		break;

	default:
error:

		retval = -EPIPE;
	}

	spin_unlock_irqrestore(&ohs900->lock, flags);
	return retval;
}

#ifdef	CONFIG_PM

static int ohs900h_bus_suspend(struct usb_hcd *hcd)
{
	// SOFs off
	pr_debug("%s\n", __func__);
	return 0;
}

static int ohs900h_bus_resume(struct usb_hcd *hcd)
{
	// SOFs on
	pr_debug("%s\n", __func__);
	return 0;
}

#else

#define	ohs900h_bus_suspend	NULL
#define	ohs900h_bus_resume	NULL

#endif

/*-------------------------------------------------------------------------*/

#ifdef STUB_DEBUG_FILE

static inline void create_debug_file(struct ohs900 *ohs900)
{
}

static inline void remove_debug_file(struct ohs900 *ohs900)
{
}

#else

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void dump_irq(struct seq_file *s, char *label, u8 mask)
{
	seq_printf(s, "%s %02x%s%s%s%s\n", label, mask,
		   (mask & OHS900_INTMASK_TRANS_DONE) ? " done" : "",
		   (mask & OHS900_INTMASK_SOFINTR) ? " sof" : "",
		   (mask & OHS900_INTMASK_INSRMV) ? " ins/rmv" : "",
		   (mask & OHS900_INTMASK_RESUME_DET) ? " rd" : "");
}

static int proc_ohs900h_show(struct seq_file *s, void *unused)
{
	struct ohs900 *ohs900 = s->private;
	struct ohs900h_ep *ep;
	unsigned i;
	u8 t;
	pr_debug("Enter static int proc_ohs900h_show(struct seq_file *s, void *unused)");
	seq_printf(s, "%s\n%s version %s\nportstatus[1] = %08x\n",
		   ohs900_to_hcd(ohs900)->product_desc,
		   hcd_name, DRIVER_VERSION, ohs900->port1);

	seq_printf(s, "insert/remove: %ld\n", ohs900->stat_insrmv);
	seq_printf(s, "current session:  done_a %ld done_b %ld "
		   "wake %ld sof %ld overrun %ld lost %ld\n\n",
		   ohs900->stat_a, ohs900->stat_b,
		   ohs900->stat_wake, ohs900->stat_sof,
		   ohs900->stat_overrun, ohs900->stat_lost);

	spin_lock_irq(&ohs900->lock);

	t = ohs900_read(ohs900, OHS900_TXLINECTLREG);

	seq_printf(s, "ctrl1 %02x%s%s%s%s\n", t,
		   (ohs900_read(ohs900, OHS900_SOFENREG)) ? " sofgen" : "", ( {
									     char
									     *s;
									     switch
									     (t
									      &
									      OHS900_TXLCTL_MASK_LINE_CTRL_BITS)
									     {
case OHS900_TXLCTL_MASK_NORMAL:
s = ""; break; case OHS900_TXLCTL_MASK_SE0:
s = " se0/reset"; break; case OHS900_TXLCTL_MASK_FS_K:
s = " FS k/resume"; break; case OHS900_TXLCTL_MASK_FS_J:
s = " FS J/resume"; break; default:
									     s =
									     " not valid ?";
									     break;};
									     s;}
		   ),
		   (t & OHS900_TXLCTL_MASK_FS_POL) ? " fs pol" : " ls pol ",
		   (t & OHS900_TXLCTL_MASK_FS_RATE) ? " fs rate" :
		   " ls rate ") ;

	dump_irq(s, "irq_enable", ohs900_read(ohs900, OHS900_IRQ_ENABLE));
	dump_irq(s, "irq_status", ohs900_read(ohs900, OHS900_IRQ_STATUS));
	seq_printf(s, "frame clocks remaining:  %d\n",
		   ohs900_read(ohs900, OHS900_SOFTMRREG) << 6);

	seq_printf(s, "A: qh%p ctl %02x sts %02x\n", ohs900->active_a,
		   ohs900_read(ohs900, OHS900_HOST_TX_CTLREG),
		   ohs900_read(ohs900, OHS900_HRXSTATREG));
	seq_printf(s, "\n");
	list_for_each_entry(ep, &ohs900->async, schedule) {
		struct urb *urb;

		seq_printf(s, "%s%sqh%p, ep%d%s, maxpacket %d"
			   " nak %d err %d\n",
			   (ep == ohs900->active_a) ? "(A) " : "",
			   (ep == ohs900->active_b) ? "(B) " : "",
			   ep, ep->epnum, ( {
					   char *s;
					   switch (ep->nextpid) {
case USB_PID_IN:
s = "in"; break; case USB_PID_OUT:
s = "out"; break; case USB_PID_SETUP:
s = "setup"; break; case USB_PID_ACK:
s = "status"; break; default:
					   s = "?"; break;};
					   s;}
			   ), ep->maxpacket, ep->nak_count, ep->error_count) ;
		list_for_each_entry(urb, &ep->hep->urb_list, urb_list) {
			seq_printf(s, "  urb%p, %d/%d\n", urb,
				   urb->actual_length,
				   urb->transfer_buffer_length);
		}
	}
	if (!list_empty(&ohs900->async))
		seq_printf(s, "\n");

	seq_printf(s, "periodic size= %d\n", PERIODIC_SIZE);

	for (i = 0; i < PERIODIC_SIZE; i++) {
		ep = ohs900->periodic[i];
		if (!ep)
			continue;
		seq_printf(s, "%2d [%3d]:\n", i, ohs900->load[i]);

		/* DUMB: prints shared entries multiple times */
		do {
			seq_printf(s,
				   "   %s%sqh%d/%p (%sdev%d ep%d%s max %d) "
				   "err %d\n",
				   (ep == ohs900->active_a) ? "(A) " : "",
				   (ep == ohs900->active_b) ? "(B) " : "",
				   ep->period, ep,
				   (ep->udev->speed == USB_SPEED_FULL)
				   ? "" : "ls ",
				   ep->udev->devnum, ep->epnum,
				   (ep->epnum == 0) ? ""
				   : ((ep->nextpid == USB_PID_IN)
				      ? "in"
				      : "out"), ep->maxpacket, ep->error_count);
			ep = ep->next;
		} while (ep);
	}

	spin_unlock_irq(&ohs900->lock);
	seq_printf(s, "\n");

	return 0;
}

static int proc_ohs900h_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_ohs900h_show, PDE(inode)->data);
}

static const file_operations proc_ops = {
	.open = proc_ohs900h_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* expect just one ohs900 per system */
static const char proc_filename[] = "driver/ohs900";

static void create_debug_file(struct ohs900 *ohs900)
{
	ohs900->pde =
	    proc_create_data(proc_filename, 0, NULL, &proc_ops, ohs900)
}

static void remove_debug_file(struct ohs900 *ohs900)
{
	if (ohs900->pde)
		remove_proc_entry(proc_filename, NULL);
}

#endif

/*-------------------------------------------------------------------------*/

static void ohs900h_stop(struct usb_hcd *hcd)
{
	struct ohs900 *ohs900 = hcd_to_ohs900(hcd);
	unsigned long flags;
	pr_debug("Enter ohs900h_stop(struct usb_hcd *hcd)");
	del_timer_sync(&hcd->rh_timer);

	spin_lock_irqsave(&ohs900->lock, flags);
	port_power(ohs900, 0);
	spin_unlock_irqrestore(&ohs900->lock, flags);
}

static int ohs900h_start(struct usb_hcd *hcd)
{
	struct ohs900 *s_ohs900 = hcd_to_ohs900(hcd);

	/* chip has been reset, VBUS power is off */
	//  pr_debug("START to urb_enqueue %p", hcd->driver->urb_enqueue);    
	// pr_debug("START to ohs900h_endpoint_disable %p", hcd->driver->endpoint_disable);

	hcd->state = HC_STATE_RUNNING;

	if (s_ohs900->board) {
		if (!device_can_wakeup(hcd->self.controller))
			device_init_wakeup(hcd->self.controller,
					   s_ohs900->board->can_wakeup);
		hcd->power_budget = s_ohs900->board->power * 2;
	}

	/* enable power and interrupts */
	port_power(s_ohs900, 1);

	/* This is necessary to make the controller detect devices
	 * that are already plugged in during initialisation. It
	 * causes the hub driver to do a reset, which triggers
	 * checking for an attached device. */
	s_ohs900->port1 |= (1 << USB_PORT_FEAT_C_CONNECTION)
	    | (1 << USB_PORT_FEAT_CONNECTION);

	return 0;
}

/*-------------------------------------------------------------------------*/

static struct hc_driver ohs900h_hc_driver = {
	.description = hcd_name,
	.hcd_priv_size = sizeof(struct ohs900),

	/*
	 * generic hardware linkage
	 */
	.irq = ohs900h_irq,
	.flags = HCD_USB11,

	.start = ohs900h_start,
	.stop = ohs900h_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ohs900h_urb_enqueue,
	.urb_dequeue = ohs900h_urb_dequeue,
	.endpoint_disable = ohs900h_endpoint_disable,

	/*
	 * periodic schedule support
	 */
	.get_frame_number = ohs900h_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ohs900h_hub_status_data,
	.hub_control = ohs900h_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend = ohs900h_bus_suspend,
	.bus_resume = ohs900h_bus_resume,
#endif
};

/*-------------------------------------------------------------------------*/

static int ohs900h_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct ohs900 *ohs900 = hcd_to_ohs900(hcd);
	struct resource *res;
	pr_debug("Enter ohs900h_remove(struct platform_device *dev)");
	remove_debug_file(ohs900);
	usb_remove_hcd(hcd);

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res)
		iounmap(ohs900->addr_reg);

	usb_put_hcd(hcd);
	return 0;
}

static int ohs900h_probe(struct platform_device *pdev)
{

	struct usb_hcd *hcd;
	struct ohs900 *s_ohs900;
	struct resource *res, *ires;
	struct ohs900_platform_data *pdata;

	int irq;
	void __iomem *addr_reg;
	int retval;
	u8 tmp;
	unsigned long irqflags;

	/* basic sanity checks first.  board-specific init logic should
	 * have initialized these three resources and probably board
	 * specific platform_data.  we don't probe for IRQs, and do only
	 * minimal sanity checking.
	 */

	dev_dbg(&pdev->dev, "driver %s, starting ohs900h_probe\n", hcd_name);
	ires = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (pdev->num_resources < 3 || !ires)
		return -ENODEV;

	irq = ires->start;
	irqflags = ires->flags & IRQF_TRIGGER_MASK;

	/* refuse to confuse usbcore */
	if (pdev->dev.dma_mask) {
		dev_dbg(&pdev->dev, "no we won't dma\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!devm_request_mem_region(&pdev->dev, res->start,
				     resource_size(res),
				     dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "Resource not available\n");
		return -EBUSY;
 	}

	addr_reg = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (addr_reg == NULL) {
		return -ENOMEM;
	}

	pdata = (struct ohs900_platform_data *)pdev->dev.platform_data;
	if (!pdata) {
		int* val;

		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct ohs900_platform_data),
			    	GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		val = (int*)of_get_property(pdev->dev.of_node, "can_wakeup",
					   NULL);
		if (!val) {
			dev_err(&pdev->dev,
				"Missing required paramter 'can_wakeup'");
			return -ENODEV;
		}
		pdata->can_wakeup = *val;

		val = (int*)of_get_property(pdev->dev.of_node, "potpg", NULL);
		if (!val) {
			dev_err(&pdev->dev, "Missing required paramter 'potpg'");
			return -ENODEV;
		}
		pdata->potpg = *val;

		val = (int*)of_get_property(pdev->dev.of_node, "power", NULL);
		if (!val) {
			dev_err(&pdev->dev, "Missing required paramter 'power'");
			return -ENODEV;
		}
		pdata->power = *val;
	}

	/* allocate and initialize hcd */
	hcd = usb_create_hcd(&ohs900h_hc_driver, &pdev->dev, pdev->name);

	if (!hcd) {
		return -ENOMEM;
	}
	hcd->rsrc_start = res->start;
	
	s_ohs900 = hcd_to_ohs900(hcd);

	pr_debug("driver %s, spin_lock_init\n", hcd_name);
	spin_lock_init(&s_ohs900->lock);
	INIT_LIST_HEAD(&s_ohs900->async);

	init_timer(&s_ohs900->timer);
	s_ohs900->timer.function = ohs900h_timer;
	s_ohs900->timer.data = (unsigned long)s_ohs900;
	s_ohs900->addr_reg = addr_reg;
	s_ohs900->board = pdata;

	spin_lock_irq(&s_ohs900->lock);
	port_power(s_ohs900, 0);
	spin_unlock_irq(&s_ohs900->lock);

	msleep(200);

	tmp = ohs900_read(s_ohs900, 0xe0);

	pr_debug("driver %s, getting hw version\n", hcd_name);
	tmp = ohs900_read(s_ohs900, OHS900_HWREVREG);

	switch (tmp) {
	case 0x7:
		hcd->product_desc = "OHS900 v0.7";
		break;
	case 0x8:
		hcd->product_desc = "OHS900 v0.8";
		break;
	case 0x10:
		hcd->product_desc = "OHS900 v1.0";
		break;
	case 0x11:
		hcd->product_desc = "OHS900 v1.1";
		break;
	case 0x12:
		hcd->product_desc = "OHS900 v1.2";
		break;
	case 0x20:
		hcd->product_desc = "OHS900 v1.3";
		break;
	default:
		/* reject other chip revisions */
		retval = -ENODEV;
		goto err_out;
	}
	pr_debug("driver %s, hw version = %d\n", hcd_name, tmp);

	//irqflags |= IRQF_SHARED;
	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED);	// | irqflags);
	if (retval != 0)
		goto err_out;

	dev_dbg(&pdev->dev, "%s, irq %d\n", hcd->product_desc, irq);

	create_debug_file(s_ohs900);
	return 0;

err_out:
	pr_debug("init error, %d\n", retval);
	usb_put_hcd(hcd);
	return retval;
}

#ifdef	CONFIG_PM

/* for this device there's no useful distinction between the controller
 * and its root hub, except that the root hub only gets direct PM calls 
 * when CONFIG_USB_SUSPEND is enabled.
 */

static int ohs900h_suspend(struct platform_device *dev)
{
	struct ohs900 *ohs900 = dev_get_drvdata(dev);
	int retval = 0;
	pr_debug("Enter ohs900h_suspend(struct platform_device *dev)");
	if (phase != SUSPEND_POWER_DOWN)
		return retval;

	if (state <= PM_SUSPEND_MEM)
		retval = ohs900h_bus_suspend(ohs900_to_hcd(ohs900));
	else
		port_power(ohs900, 0);
	if (retval == 0)
		dev->power.power_state = state;
	return retval;
}

static int ohs900h_resume(struct platform_device *dev)
{
	struct ohs900 *ohs900 = dev_get_drvdata(dev);
	pr_debug("Enter ohs900h_resume(struct platform_device *dev)");
	if (phase != RESUME_POWER_ON)
		return 0;

	/* with no "check to see if VBUS is still powered" board hook,
	 * let's assume it'd only be powered to enable remote wakeup.
	 */
	if (!ohs900->port1 || !ohs900_to_hcd(ohs900)->can_wakeup) {
		ohs900->port1 = 0;
		port_power(ohs900, 1);
		return 0;
	}

	return ohs900h_bus_resume(ohs900_to_hcd(ohs900));
}

#else

#define	ohs900h_suspend	NULL
#define	ohs900h_resume	NULL

#endif

static struct of_device_id ocores_ohs900_match[] = {
	{
	 .compatible = "opencores,ohs900-ocores",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, ocores_ohs900_match);

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:ohs900");
struct platform_driver ohs900h_driver = {
	.probe = ohs900h_probe,
	.remove = ohs900h_remove,
	.suspend = ohs900h_suspend,
	.resume = ohs900h_resume,
	.driver = {
		   .name = hcd_name,
		   .owner = THIS_MODULE,
		   .of_match_table = ocores_ohs900_match,
	},
};

/*-------------------------------------------------------------------------*/

static int __init ohs900h_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_debug("driver %s, %s\n", hcd_name, DRIVER_VERSION);
	return platform_driver_register(&ohs900h_driver);
}

module_init(ohs900h_init);

static void __exit ohs900h_cleanup(void)
{
	platform_driver_unregister(&ohs900h_driver);

}

module_exit(ohs900h_cleanup);
