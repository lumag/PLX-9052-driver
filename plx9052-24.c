/*======================================================================

    Device driver for PLX9052 PCI-to-PCMCIA bridges.
    Version 0.1.5.

    (C) 2003 Pavel Roskin <proski@gnu.org>

    The contents of this file are subject to the Mozilla Public
    License Version 1.1 (the "License"); you may not use this file
    except in compliance with the License. You may obtain a copy of
    the License at http://www.mozilla.org/MPL/

    Software distributed under the License is distributed on an "AS
    IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
    implied. See the License for the specific language governing
    rights and limitations under the License.

    Alternatively, the contents of this file may be used under the
    terms of the GNU General Public License version 2 (the "GPL"), in
    which case the provisions of the GPL are applicable instead of the
    above.  If you wish to allow the use of your version of this file
    only under the terms of the GPL and not to allow others to use
    your version of this file under the MPL, indicate your decision
    by deleting the provisions above and replace them with the notice
    and other provisions required by the GPL.  If you do not delete
    the provisions above, a recipient may use your version of this
    file under either the MPL or the GPL.

======================================================================*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/tqueue.h>
#include <linux/version.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/bitops.h>
#include <asm/system.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/ss.h>
#include <pcmcia/cs.h>

/*====================================================================*/

/* Module parameters */

MODULE_AUTHOR("Pavel Roskin <proski@gnu.org>");
MODULE_DESCRIPTION("PLX9052 PCMCIA socket driver");
MODULE_LICENSE("Dual MPL/GPL");

/*====================================================================*/

#define DEVICE_NAME "plx9052"

#define PLX_IOWIN_MIN	0x04	/* Minimal I/O window */
#define PLX_IOWIN_MAX	0x1000	/* Maximal I/O window */
#define PLX_MEMWIN_MIN	0x10	/* Minimal memory window */
#define PLX_MEMWIN_MAX	0x80000	/* Maximal memory window */

#define PLX_CIS_START	0x03000000	/* Start of CIS in PLX local memory */

/*
 * PLX9052 has 4 local address spaces.
 * The driver allocates them dynamically.
 */
#define PLX_AREAS	4	/* Number of local address spaces (areas) */
#define PLX_AREA_USED	0x20	/* Area is used */
#define PLX_AREA_IO	0x10	/* Area is used for I/O window */
#define PLX_AREA_MAP_MASK	0x0f	/* Map number */


/* PCI base address for corresponding local address space */
#define PLX_PCI_BASE(x)	(PCI_BASE_ADDRESS_2 + (x << 2))


/* PLX9052 registers */

/* Local address space range (0 <= x < PLX_AREAS) */
#define PLX_LASRR(x)	(0x00 + (x << 2))
#define PLX_LASRR_IO	1	/* Range is used as I/O space */
#define PLX_LAS_MEMMASK	0x0ffffff0	/* Mask for memory address lines */
#define PLX_LAS_IOMASK	0x0ffffffc	/* Mask for I/O address lines */

/* Local address space base address (0 <= x < PLX_AREAS) */
#define PLX_LASBA(x)	(0x14 + (x << 2))
#define PLX_LASBA_DISABLE 0	/* Disable address decoding */
#define PLX_LASBA_ENABLE  1	/* Enable address decoding */

/* Local address space bus region descriptors (0 <= x < PLX_AREAS) */
#define PLX_LASBRD(x)	(0x28 + (x << 2))
#define PLX_LASBRD_WIDTH_MASK	0x00c00000
#define PLX_LASBRD_WIDTH_8	0x00000000
#define PLX_LASBRD_WIDTH_16	0x00400000
#define PLX_LASBRD_WIDTH_32	0x00800000
#define PLX_LASBRD_WS_MASK	0x003fffc0
#define PLX_LASBRD_WS0		0x00000000
#define PLX_LASBRD_WS1		0x0010a840

/* Chip select base address (0 <= x < PLX_AREAS) */
#define PLX_CSBASE(x)	(0x3c + (x << 2))
#define PLX_CSBASE_ENABLE	1

#define PLX_INTCSR	0x4c	/* Interrupt Control and Status Register */

#define PLX_INT1_ENABLE	0x40	/* Interrupt 1 Enable bit */
#define PLX_INT1_ACTIVE	0x04	/* Interrupt 1 Active bit */

#define PLX_CNTRL	0x50	/* Control register */
#define PLX_CNTRL_RESET	0x40000000	/* Reset local bus */


/* One socket per card, but we may have more than one card */
#define PCMCIA_SOCKETS_NO 8

#ifdef __IN_PCMCIA_PACKAGE__
socket_state_t dead_socket = {
	0, SS_DETECT, 0, 0, 0
};
#endif


typedef struct socket_info_t {
	void (*handler) (void *info, u_int events);
	void *info;

	socket_state_t state;
	struct pci_dev *pdev;	/* PCI device */

	/* PCI resource 1, PLX9052 registers */
	unsigned int plxctl_addr;
	unsigned int plxctl_len;	/* should be 0x80 */

	/* PCI resource 2, mapped to PCMCIA memory space */
	unsigned long mem_phys;
	unsigned long mem_len;	/* should be 0x1000 */
	u8 *mem_virt;

	char area_used[PLX_AREAS];	/* Used local address spaces */

	/* Event processing */
	u_int event;
	spinlock_t event_lock;
	struct tq_struct event_work;
} socket_info_t;

static socket_info_t socket_table[PCMCIA_SOCKETS_NO];
static int socket_count;

/* Forward declarations */
static void plx9052_close(struct pci_dev *pdev);
static int plx9052_set_socket(unsigned int sock, socket_state_t * state);


/* Check that an area is size-aligned.
 * Return 0 for good areas, -1 for bad ones.  */
static int plx9052_align_check(u32 base, u32 len)
{
	int power = 0;
	int l = len;

	if (!len)
		return -1;

	while (l >>= 1)
		power++;

	if (len != (1 << power))
		return -1;

	if (base & ((1 << power) - 1))
		return -1;

	return 0;
}


static inline void plx9052_outl(socket_info_t *socket, u32 val,
				ioaddr_t addr)
{
	outl_p(val, socket->plxctl_addr + addr);
}


static inline u32 plx9052_inl(socket_info_t *socket, ioaddr_t addr)
{
	return inl_p(socket->plxctl_addr + addr);
}


static inline void plx9052_enable_irq(socket_info_t *socket)
{
	u32 reg;

	reg = plx9052_inl(socket, PLX_INTCSR);
	if (!(reg & PLX_INT1_ENABLE)) {
		reg |= PLX_INT1_ENABLE;
		plx9052_outl(socket, reg, PLX_INTCSR);
	}
}


static inline void plx9052_disable_irq(socket_info_t *socket)
{
	u32 reg;

	reg = plx9052_inl(socket, PLX_INTCSR);
	if (reg & PLX_INT1_ENABLE) {
		reg &= ~PLX_INT1_ENABLE;
		plx9052_outl(socket, reg, PLX_INTCSR);
	}
}


static inline int plx9052_irq_active(socket_info_t *socket)
{
	u32 reg;

	reg = plx9052_inl(socket, PLX_INTCSR);
	return (reg & PLX_INT1_ACTIVE);
}


/* Hack - determine card presence by the first byte of CIS */
static inline int plx9052_card_present(socket_info_t *socket)
{
	return (((volatile u8 *) socket->mem_virt)[0] == 1);
}


static void plx9052_event(socket_info_t *socket)
{
	u_int event;

	if (!socket->handler)
		return;

	spin_lock_irq(&socket->event_lock);
	event = socket->event;
	socket->event = 0;
	spin_unlock_irq(&socket->event_lock);

	if (!event)
		return;

	/* To make sure it'socket an insertion, wait 1 second before CIS is ready.
	 * If the card is currently active, report ejection immediately.  */
	if (!(socket->state.flags & SS_OUTPUT_ENA)) {
		current->state = TASK_INTERRUPTIBLE;
		schedule_timeout(HZ);
	}

	socket->handler(socket->info, event);

	if (!(socket->state.flags & SS_RESET))
		plx9052_enable_irq(socket);
}


static int plx9052_get_area(socket_info_t *socket, int is_io, int map)
{
	int i;

	/* Area 0 is used internally for CIS */
	for (i = 1; i < PLX_AREAS; i++) {
		if (!socket->area_used[i]) {
			socket->area_used[i] = PLX_AREA_USED | map
			    | (is_io ? PLX_AREA_IO : 0);
			return i;
		}
	}

	return -1;
}


static void plx9052_disable_areas(socket_info_t *socket, int is_io,
				  int map)
{
	int i;
	int type = PLX_AREA_USED | map | (is_io ? PLX_AREA_IO : 0);

	/* Note that there may be more than one area for the map */
	for (i = 1; i < PLX_AREAS; i++) {
		if (socket->area_used[i] == type) {
			socket->area_used[i] = 0;
			plx9052_outl(socket, PLX_LASBA_DISABLE,
				     PLX_LASBA(i));
		}
	}
}


/* program a size-aligned area */
static int plx9052_program_area(socket_info_t *socket, int is_io, int map,
				unsigned long sysbase,
				unsigned long syslen, u32 cardbase,
				unsigned int flags)
{
	u32 brd;
	u32 mask;
	int area;

	area = plx9052_get_area(socket, is_io, map);
	if (area < 0) {
		printk(KERN_ERR "No free areas left\n");
		return -EINVAL;
	}

	pci_write_config_dword(socket->pdev, PLX_PCI_BASE(area),
			       sysbase | is_io);

	mask = ~(syslen - 1);
	if (is_io)
		mask = (mask & PLX_LAS_IOMASK) | PLX_LASRR_IO;
	else
		mask &= PLX_LAS_MEMMASK;

	brd = plx9052_inl(socket, PLX_LASBRD(area));
	brd &= ~PLX_LASBRD_WIDTH_MASK;
	if (flags & MAP_16BIT)
		brd |= PLX_LASBRD_WIDTH_16;

	brd &= ~PLX_LASBRD_WS_MASK;
	if (!(flags & MAP_0WS))
		brd |= PLX_LASBRD_WS1;

	plx9052_outl(socket, brd, PLX_LASBRD(area));

	plx9052_outl(socket, mask, PLX_LASRR(area));
	plx9052_outl(socket, cardbase | PLX_LASBA_ENABLE, PLX_LASBA(area));

	return 0;
}


static int plx9052_init(unsigned int sock)
{
	socket_info_t *socket = &socket_table[sock];

#if 0
	/* Dump all registers */
	int i;
	for (i = 0; i < 0x20; i++) {
		printk("%08x ", plx9052_inl(socket, i << 2));
		if ((i & 7) == 7)
			printk("\n");
	}
#endif

	/* Area 0 is used to access CIS from the driver */
	plx9052_outl(socket, PLX_LASBA_ENABLE | PLX_CIS_START,
		     PLX_LASBA(0));
	socket->area_used[0] = 1;

	plx9052_outl(socket,
		     PLX_CIS_START | (PLX_MEMWIN_MAX >> 1) |
		     PLX_CSBASE_ENABLE, PLX_CSBASE(0));
	plx9052_outl(socket, (PLX_IOWIN_MAX >> 1) | PLX_CSBASE_ENABLE,
		     PLX_CSBASE(1));

	plx9052_enable_irq(socket);
	socket->state.csc_mask |= SS_DETECT;

	return 0;
}


#ifdef __IN_PCMCIA_PACKAGE__
static int plx9052_register_callback(unsigned int sock,
				     ss_callback_t *call)
{
	socket_info_t *socket = &socket_table[sock];

	if (call == NULL) {
		socket->handler = NULL;
		MOD_DEC_USE_COUNT;
	} else {
		MOD_INC_USE_COUNT;
		socket->handler = call->handler;
		socket->info = call->info;
	}

	return 0;
}

#else

static int plx9052_register_callback(unsigned int sock,
				     void (*handler) (void *,
						      unsigned int),
				     void *info)
{
	socket_info_t *socket = &socket_table[sock];

	socket->handler = handler;
	socket->info = info;

	if (handler == NULL) {
		MOD_DEC_USE_COUNT;
	} else {
		MOD_INC_USE_COUNT;
	}

	return 0;
}


static int plx9052_suspend(unsigned int sock)
{
	plx9052_set_socket(sock, &dead_socket);
	return 0;
}
#endif


static int plx9052_inquire_socket(unsigned int sock, socket_cap_t *cap)
{
	socket_info_t *socket = &socket_table[sock];

	memset(cap, 0, sizeof(socket_cap_t));

	/* only 16-bit cards, size-aligned */
	cap->features |=
	    SS_CAP_PCCARD | SS_CAP_MEM_ALIGN | SS_CAP_PAGE_REGS;
	cap->map_size = PLX_MEMWIN_MIN;	/* minimum window size */
	cap->pci_irq = socket->pdev->irq;

	return 0;
}


static int plx9052_get_status(unsigned int sock, u_int *value)
{
	socket_info_t *socket = &socket_table[sock];

	*value = 0;

	if (plx9052_card_present(socket))
		*value |= SS_READY | SS_POWERON | SS_IOCARD | SS_DETECT;

	return 0;
}


static int plx9052_get_socket(unsigned int sock, socket_state_t *state)
{
	*state = socket_table[sock].state;
	return 0;
}


static int plx9052_set_socket(unsigned int sock, socket_state_t *state)
{
	u32 reg;
	socket_info_t *socket = &socket_table[sock];

	if (state->flags & SS_RESET) {
		plx9052_disable_irq(socket);
		reg = plx9052_inl(socket, PLX_CNTRL);
		reg |= PLX_CNTRL_RESET;
		plx9052_outl(socket, PLX_CNTRL, reg);
	} else {
		reg = plx9052_inl(socket, PLX_CNTRL);
		reg &= ~PLX_CNTRL_RESET;
		plx9052_outl(socket, PLX_CNTRL, reg);
		plx9052_enable_irq(socket);
	}

	socket->state = *state;
	return 0;
}


static int plx9052_set_io_map(unsigned int sock, struct pccard_io_map *io)
{
	socket_info_t *socket = &socket_table[sock];
	unsigned int len;	/* requested length */
	unsigned int len2;	/* length adjusted to power of two */
	unsigned int start2;	/* start adjusted to power of two */
	unsigned int split;	/* split point for unaligned windows */
	unsigned int tmp;
	int err;

	/* Disable mapping before changing it */
	plx9052_disable_areas(socket, 1, io->map);

	if (!(io->flags & MAP_ACTIVE))
		return 0;

	len = io->stop + 1 - io->start;
	if (len > PLX_IOWIN_MAX) {
		printk(KERN_ERR
		       "Requested I/O area 0x%x-0x%x is too long\n",
		       io->start, io->stop);
		return -EINVAL;
	}

	/* Simplest case - size aligned window */
	if (!plx9052_align_check(io->start, len)) {
		err =
		    plx9052_program_area(socket, 1, io->map, io->start,
					 len, 0, io->flags);
		return err;
	}

	/* Find the highest address line that needs to be opened */
	tmp = io->stop ^ io->start;
	for (len2 = PLX_IOWIN_MIN; len2 <= PLX_IOWIN_MAX; len2 <<= 1) {
		if (len2 > tmp)
			break;
	}

	/* Split the requested window into at most two size-aligned windows */
	start2 = ~(len2 - 1) & io->start;
	split = start2 + (len2 >> 1);

	if (plx9052_align_check(io->start, split - io->start) ||
	    plx9052_align_check(split, io->stop + 1 - split)) {
		printk(KERN_ERR
		       "I/O area 0x%x-0x%x is too badly unaligned\n",
		       io->start, io->stop);
		return -ENOTSUPP;
	}

	err = plx9052_program_area(socket, 1, io->map, io->start,
				   split - io->start, 0, io->flags);

	if (err)
		return err;

	err = plx9052_program_area(socket, 1, io->map, split,
				   io->stop + 1 - split, 0, io->flags);

	return err;
}


static int plx9052_set_mem_map(unsigned int sock,
			       struct pccard_mem_map *mem)
{
	socket_info_t *socket = &socket_table[sock];
	unsigned long len;	/* requested length */
	int err;

	/* Disable mapping before changing it */
	plx9052_disable_areas(socket, 0, mem->map);

	if (!(mem->flags & MAP_ACTIVE))
		return 0;

	/* Memory allocation in the first megabyte is problematic on
	 * some machines with Intel chipset.  */
	if (mem->sys_start < 0x100000)
		return -EINVAL;

	len = mem->sys_stop + 1 - mem->sys_start;
	if (len > PLX_MEMWIN_MAX) {
		printk(KERN_ERR "Memory map 0x%lx-0x%lx is too long\n",
		       mem->sys_start, mem->sys_stop);
		return -EINVAL;
	}

	if (plx9052_align_check(mem->sys_start, len)) {
		printk(KERN_ERR
		       "Memory map 0x%lx-0x%lx is not size-aligned\n",
		       mem->sys_start, mem->sys_stop);
		return -EINVAL;
	}

	err =
	    plx9052_program_area(socket, 0, mem->map, mem->sys_start, len,
				 mem->card_start | PLX_CIS_START,
				 mem->flags);

	return err;
}


static void plx9052_proc_setup(unsigned int sock,
			       struct proc_dir_entry *base)
{
	return;
}


static void plx9052_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	socket_info_t *socket = (socket_info_t *) dev_id;

	if (!plx9052_irq_active(socket))
		return;

	if (!(socket->state.csc_mask & SS_DETECT))
		return;

	if (!plx9052_card_present(socket)) {
		socket->event |= SS_DETECT;
		plx9052_disable_irq(socket);
		schedule_task(&socket->event_work);
	}

	return;
}


#ifdef __IN_PCMCIA_PACKAGE__
typedef int (*subfn_t) (unsigned int, void *);

static subfn_t service_table[] = {
	(subfn_t) & plx9052_register_callback,
	(subfn_t) & plx9052_inquire_socket,
	(subfn_t) & plx9052_get_status,
	(subfn_t) & plx9052_get_socket,
	(subfn_t) & plx9052_set_socket,
	NULL,
	(subfn_t) & plx9052_set_io_map,
	NULL,
	(subfn_t) & plx9052_set_mem_map,
};

#define NFUNC (sizeof(service_table)/sizeof(subfn_t))

static int plx9052_operations(u_int lsock, u_int cmd, void *arg)
{
	subfn_t func;

	if (cmd >= sizeof(service_table) / sizeof(subfn_t))
		return -EINVAL;

	func = service_table[cmd];
	if (!func)
		return -EINVAL;

	return func(lsock, arg);
}

#else

static struct pccard_operations plx9052_operations = {
	.init = plx9052_init,
	.suspend = plx9052_suspend,
	.register_callback = plx9052_register_callback,
	.inquire_socket = plx9052_inquire_socket,
	.get_status = plx9052_get_status,
	.get_socket = plx9052_get_socket,
	.set_socket = plx9052_set_socket,
	.set_io_map = plx9052_set_io_map,
	.set_mem_map = plx9052_set_mem_map,
	.proc_setup = plx9052_proc_setup,
};
#endif


static int plx9052_probe(struct pci_dev *pdev,
			 const struct pci_device_id *ent)
{
	int err = 0;
	socket_info_t *socket = NULL;

	if (socket_count >= PCMCIA_SOCKETS_NO) {
		printk(KERN_ERR
		       "plx9052: at most %d sockets are supported\n",
		       PCMCIA_SOCKETS_NO);
		return -ENODEV;
	}

	socket = &socket_table[socket_count];
	socket_count++;
	memset(socket, 0, sizeof(socket_info_t));

	socket->pdev = pdev;
	pci_set_drvdata(pdev, socket);

	spin_lock_init(&socket->event_lock);
	INIT_TQUEUE(&socket->event_work, (void (*)(void *)) plx9052_event,
		    socket);

	err = pci_enable_device(pdev);
	if (err)
		return -EIO;

	/* Resource 1 is control registers of PLX9052 */
	socket->plxctl_addr = pci_resource_start(pdev, 1);
	socket->plxctl_len = pci_resource_len(pdev, 1);
	if (!request_region
	    (socket->plxctl_addr, socket->plxctl_len, DEVICE_NAME)) {
		printk(KERN_ERR "plx9052: I/O at 0x%x-0x%x busy\n",
		       socket->plxctl_addr,
		       socket->plxctl_addr + socket->plxctl_len - 1);
		socket->plxctl_addr = 0;
		err = -EBUSY;
		goto fail;
	}

	/* Resource 2 is mapped to the PCMCIA memory space, starting with CIS */
	socket->mem_phys = pci_resource_start(pdev, 2);
	socket->mem_len = pci_resource_len(pdev, 2);
	if (!request_mem_region
	    (socket->mem_phys, socket->mem_len, DEVICE_NAME)) {
		printk(KERN_ERR "plx9052: memory at 0x%lx-0x%lx busy\n",
		       socket->mem_phys,
		       socket->mem_phys + socket->mem_len - 1);
		socket->mem_phys = 0;
		err = -EBUSY;
		goto fail;
	}

	socket->mem_virt = ioremap(socket->mem_phys, socket->mem_len);
	if (!socket->mem_virt) {
		printk(KERN_ERR
		       "plx9052: cannot map memory at 0x%lx-0x%lx\n",
		       socket->mem_phys,
		       socket->mem_phys + socket->mem_len - 1);
		err = -ENOMEM;
		goto fail;
	}

	err =
	    request_irq(pdev->irq, plx9052_interrupt, SA_SHIRQ,
			DEVICE_NAME, socket);
	if (err) {
		printk(KERN_ERR "plx9052: cannot allocate IRQ %d.\n",
		       pdev->irq);
		err = -EBUSY;
		goto fail;
	}

	printk(KERN_INFO "plx9052: Socket %d enabled, IRQ %d\n",
	       socket_count, socket->pdev->irq);

	return 0;		/* succeeded */

      fail:
	plx9052_close(pdev);

	return err;
}


static void plx9052_close(struct pci_dev *pdev)
{
	socket_info_t *socket = pci_get_drvdata(pdev);

	if (pdev->irq)
		free_irq(pdev->irq, socket);

	pci_set_drvdata(pdev, NULL);
	socket->pdev = NULL;

	if (socket->plxctl_addr)
		release_region(socket->plxctl_addr, socket->plxctl_len);

	if (socket->mem_virt)
		iounmap(socket->mem_virt);

	if (socket->mem_phys)
		release_mem_region(socket->mem_phys, socket->mem_len);

	pci_disable_device(pdev);
}


/* PCI ID table, from orinoco_plx.c */
static struct pci_device_id plx9052_pci_id_table[] __devinitdata = {
	{0x111a, 0x1023, PCI_ANY_ID, PCI_ANY_ID,},	/* Siemens SpeedStream SS1023 */
	{0x1385, 0x4100, PCI_ANY_ID, PCI_ANY_ID,},	/* Netgear MA301 */
	{0x15e8, 0x0130, PCI_ANY_ID, PCI_ANY_ID,},	/* Correga  - does this work? */
	{0x1638, 0x1100, PCI_ANY_ID, PCI_ANY_ID,},	/* SMC EZConnect SMC2602W,
							   Eumitcom PCI WL11000,
							   Addtron AWA-100 */
	{0x16ab, 0x1100, PCI_ANY_ID, PCI_ANY_ID,},	/* Global Sun Tech GL24110P */
	{0x16ab, 0x1101, PCI_ANY_ID, PCI_ANY_ID,},	/* Reported working, but unknown */
	{0x16ab, 0x1102, PCI_ANY_ID, PCI_ANY_ID,},	/* Linksys WDT11 */
	{0x16ec, 0x3685, PCI_ANY_ID, PCI_ANY_ID,},	/* USR 2415 */
	{0xec80, 0xec00, PCI_ANY_ID, PCI_ANY_ID,},	/* Belkin F5D6000 tested by
							   Brendan W. McAdams <rit@jacked-in.org> */
	{0x10b7, 0x7770, PCI_ANY_ID, PCI_ANY_ID,},	/* 3Com AirConnect PCI tested by
							   Damien Persohn <damien@persohn.net> */
	{0,},
};

MODULE_DEVICE_TABLE(pci, plx9052_pci_id_table);


static struct pci_driver plx9052_driver = {
	.name = DEVICE_NAME,
	.id_table = plx9052_pci_id_table,
	.probe = plx9052_probe,
	.remove = plx9052_close,
};


static int __init init_plx9052(void)
{
	pci_module_init(&plx9052_driver);

	if (register_ss_entry(socket_count, &plx9052_operations) != 0) {
		printk(KERN_NOTICE
		       "plx9052: Unable to register sockets\n");
		return -ENODEV;
	}

	return 0;
}


extern void __exit exit_plx9052(void)
{
	unregister_ss_entry(&plx9052_operations);
	pci_unregister_driver(&plx9052_driver);
}


module_init(init_plx9052);
module_exit(exit_plx9052);
