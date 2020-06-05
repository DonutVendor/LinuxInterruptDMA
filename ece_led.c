/*
 * Devon Mickels
 * 6/5/2020
 * ECE 373
 *
 * Interrupts and Descriptors
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>

#define DEVCNT 1
#define DEVNAME "ece_led"

#define E1000_RECEIVE_BUF  0x00100
#define E1000_RECEIVE_BASE_LOW 0x02800
#define E1000_RECEIVE_BASE_HIGH 0x02804
#define E1000_RECEIVE_LENGTH 0x02808
#define E1000_RECEIVE_HEAD 0x2810
#define E1000_RECEIVE_TAIL 0x2818
#define E1000_DETECT_INTERRUPT 0x02C00

#define E1000_LEDCTL	0x00E00	/* LED Control - RW */
 
#define REG_MASK_LED0 0xFFFFFFF0
#define REG_MASK_LED1 0xFFFFF0FF
#define REG_MASK_LED2 0xFFF0FFFF
#define REG_MASK_LED3 0xF0FFFFFF
#define LED_ON 0b1110
#define LED_OFF 0b1111
 
#define GOOD_PACKETS_RECEIVED_REG 0x04074
 
#define E1000_LEDCTL_LED0_SHIFT		0
#define E1000_LEDCTL_LED1_SHIFT		8
#define E1000_LEDCTL_LED2_SHIFT		16
#define E1000_LEDCTL_LED3_SHIFT		24

//Func declarations
static int ece_led_open(struct inode *inode, struct file *file);
static int ece_led_close(struct inode *inode, struct file *file);
static int ece_led_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static int ece_led_tx_setup(void);
static int __init ece_led_init_module(void);
static void __exit ece_led_exit_module(void);
static void ece_led_remove(struct pci_dev *pdev);
static ssize_t ece_led_read(struct file *file, char __user *buf, size_t len, loff_t *offset);
static ssize_t ece_led_write(struct file *file, const char __user *buf, size_t len, loff_t *offset);
static irqreturn_t ece_irq_handler(int irq, void *data);
static void turnOnLEDs(void);
static void turnOffLEDs(void);

//End Func declarations

static const struct pci_device_id ece_led_pci_tbl[] = {
	{PCI_DEVICE(0x8086, 0x100e)},
	{},
};

static struct mydev_dev {
	struct cdev my_cdev;
	
	int syscall_val;
} mydev;

static struct pci_driver ece_led_driver = {
	.name = DEVNAME,
	.id_table = ece_led_pci_tbl,
	.probe = ece_led_probe,
	.remove = ece_led_remove
};

static struct myPCI_dev {
	struct pci_dev *pdev;
	void *hw_addr;
} myPCI_dev;

static struct ece_led_desc{
	_le64 buffer_addr;
	union{
		_le32 data;
		struct{
			_le16 length;
			u8 cso;
			u8 cmd;
		} flags;
	} lower;
	union{
		_le32 data;
		struct{
			u8 status;
			u8 css;
			_le16 special;
		} fields;
	} upper;
};

struct ece_led_desc[16] descriptors;

/* File operations for our device */
static struct file_operations mydev_fops = {
	.owner = THIS_MODULE,
	.open = ece_led_open,
	.release = ece_led_close,
	.read = ece_led_read,
	.write = ece_led_write,
};

MODULE_DEVICE_TABLE(pci, ece_led_pci_tbl);

static dev_t mydev_node;
static struct class *c1;

//Device registration handlers

static int ece_led_open(struct inode *inode, struct file *file)
{

	printk(KERN_INFO "Successfully opened ece_led!\n");

	return 0;
}

static int ece_led_close(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "Successfully closed ece_led!\n");

	return 0;
}

static int ece_led_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int err;
	unsigned long bars;
	
	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	err = pci_enable_device(pdev);
	
	if(err)
		return err;
	
	err = pci_request_selected_regions(pdev, bars,  DEVNAME);
	if(err)
		return err;
	
	myPCI_dev.hw_addr  = pci_ioremap_bar(pdev, 0);
	
	ece_led_tx_setup();
	
	printk(KERN_INFO "ece_led successfully registered\n");
	
	return 0;
}

static int ece_led_tx_setup(void){
	
	int err;
	//request an IRQ
	err = request_irq(myPCI_dev->pdev->irq, ece_irq_handler, 0, "ece_int", data); //Data is the data passed into IRQ handler
	
	return err;
}

static irqreturn_t ece_irq_handler(int irq, void *data){
	u32 cause;
	
	cause = readl(myPCI_dev->hw_addr + REG_IRQ_CAUSE);
	
	turnOnLEDs();
	
	switch(cause){
		case 0x01: printk( KERN_INFO "Test\n");
			break;
		default:
			break;
	}
	
	return IRQ_HANDLED;
}

static void turnOnLEDs(void){
	u32 ledctl;
	
	//Read value of LEDS
	ledctl = readl(myPCI_dev.hw_addr + E1000_LEDCTL);
	
	ledctl = (ledctl & REG_MASK_LED0) | LED_ON << E1000_LEDCTL_LED0_SHIFT);
	ledctl = (ledctl & REG_MASK_LED2) | LED_ON << E1000_LEDCTL_LED2_SHIFT);
	
	writel( ledctl, myPCI_dev.hw_addr  + E1000_LEDCTL);
}

static void turnOffLEDs(void){
	u32 ledctl;
	
	//Read value of LEDS
	ledctl = readl(myPCI_dev.hw_addr + E1000_LEDCTL);
	
	ledctl = (ledctl & REG_MASK_LED0) | LED_OFF << E1000_LEDCTL_LED0_SHIFT);
	ledctl = (ledctl & REG_MASK_LED2) | LED_OFF << E1000_LEDCTL_LED2_SHIFT);
	
	writel( ledctl, myPCI_dev.hw_addr  + E1000_LEDCTL);
}

static int __init ece_led_init_module(void)
{
	int ret;
	struct device *dev_ret;
	
	ret = pci_register_driver(&ece_led_driver);
	
	//Char Driver Stuff
	printk(KERN_INFO "ece_led module loading... ");

	if (alloc_chrdev_region(&mydev_node, 0, DEVCNT, DEVNAME)) {
		printk(KERN_ERR "alloc_chrdev_region() failed!\n");
		return -1;
	}

	printk(KERN_INFO "Allocated %d devices at major: %d\n", DEVCNT,
	       MAJOR(mydev_node));
		   
	c1 = class_create(THIS_MODULE, DEVNAME);
	
	dev_ret = device_create(c1, NULL, mydev_node, NULL, DEVNAME);

	/* Initialize the character device and add it to the kernel */
	cdev_init(&mydev.my_cdev, &mydev_fops);
	mydev.my_cdev.owner = THIS_MODULE;

	if (cdev_add(&mydev.my_cdev, mydev_node, DEVCNT)) {
		printk(KERN_ERR "cdev_add() failed!\n");
		/* clean up chrdev allocation */
		unregister_chrdev_region(mydev_node, DEVCNT);

		return -1;
	}

	return ret;
}

module_init(ece_led_init_module);

static void __exit ece_led_exit_module(void)
{	
	free_irq(myPCI_dev->pdev->irq, data);

	//Exit device
	cdev_del(&mydev.my_cdev);
	
	device_destroy(c1, mydev_node);
	class_destroy(c1);
	
	pci_unregister_driver(&ece_led_driver);
	
	unregister_chrdev_region(mydev_node, DEVCNT);

	printk(KERN_INFO "ece_led module unloaded!\n");
}

module_exit(ece_led_exit_module);

static void ece_led_remove(struct pci_dev *pdev)
{

	iounmap(myPCI_dev.hw_addr);
	
	pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	
	printk(KERN_INFO "Successfully unregistered ece_led\n");
}

static ssize_t ece_led_read(struct file *file, char __user *buf, size_t len, loff_t *offset)
{
	int ret; //Get a local kernel buffer sert aside

	/* Make sure our user wasn't bad... */
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}
	
	if (copy_to_user(buf, &blink_rate, sizeof(int))) {
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(int);
	*offset += sizeof(int);

	/* Good to go, so printk the thingy */
	printk(KERN_INFO "User got from us %X\n", blink_rate);

out:
	return ret;
}

static ssize_t ece_led_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
	/* Have local kernel memory ready */
	char *kern_buf;
	int ret;
	long value;
	/* Make sure our user isn't bad... */
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}
	
	//Get some memory to copy into...
	kern_buf = kmalloc(len, GFP_KERNEL);

	 //...and make sure it's good to go
	if (!kern_buf) {
		ret = -ENOMEM;
		goto out;
	}

	/* Copy from the user-provided buffer
	if (copy_from_user(&kern_buf, buf, len)) {
		ret = -EFAULT;
		goto mem_out;
	}*/
	
	if(kstrtol_from_user(buf, len, 10, &value)){
		ret = -EFAULT;
		goto mem_out;
	}
	
	//kstrtol(kern_buf, 10, &value);
	
	if(value > 0){
		blink_rate = value;
	}else{
		ret = -EINVAL;
		goto out;
	}
	
	ret = len;

	/* print what userspace gave us */
	printk(KERN_INFO "Userspace wrote \"%ld\" to us\n", value);

mem_out:
	kfree(kern_buf);
out:
	return ret;
}


MODULE_AUTHOR("Devon Mickels");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
