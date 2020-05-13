/*
 * Devon Mickels
 * 4/20/2020
 * ECE 373
 *
 * Char Driver
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/pci.h>

#define DEVCNT 1
#define DEVNAME "testPCIDriver"

#define LEDOFFSET 0xE00

//Func declarations
static int testPCIDriver_open(struct inode *inode, struct file *file);
static int testPCIDriver_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static int __init testPCIDriver_init_module(void);
static void __exit testPCIDriver_exit_module(void);
static void testPCIDriver_remove(struct pci_dev *pdev);
static ssize_t testPCIDriver_read(struct file *file, char __user *buf, size_t len, loff_t *offset);
static ssize_t testPCIDriver_write(struct file *file, const char __user *buf, size_t len, loff_t *offset);

//End Func declarations

static const struct pci_device_id testPCIDriver_pci_tbl[] = {
	{PCI_DEVICE(0x8086, 0x100e)},
	{},
};

static struct mydev_dev {
	struct cdev my_cdev;
	
	u32 syscall_val;
} mydev;

static struct pci_driver testPCIDriver_driver = {
	.name = DEVNAME,
	.id_table = testPCIDriver_pci_tbl,
	.probe = testPCIDriver_probe,
	.remove = testPCIDriver_remove
};

static struct myPCI_dev {
	struct pci_dev *pdev;
	void *hw_addr;
} myPCI_dev;

/* File operations for our device */
static struct file_operations mydev_fops = {
	.owner = THIS_MODULE,
	.open = testPCIDriver_open,
	.read = testPCIDriver_read,
	.write = testPCIDriver_write,
};

MODULE_DEVICE_TABLE(pci, testPCIDriver_pci_tbl);

static dev_t mydev_node;

static int testPCIDriver_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "Successfully opened testPCIDriver!\n");\

	return 0;
}

static int testPCIDriver_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int bars, err;
	
	bars = pci_select_bars(pdev, IORESOURCE_MEM | IORESOURCE_IO);
	err = pci_enable_device(pdev);
	
	if(err)
		return err;
	
	err = pci_request_selected_regions(pdev, bars,  DEVNAME);
	if(err)
		return err;
	
	myPCI_dev.hw_addr  = pci_ioremap_bar(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	
	printk(KERN_INFO "testPCIDriver successfully registered\n");
	
	return 0;
}

static int __init testPCIDriver_init_module(void)
{
	int ret;
	ret = pci_register_driver(&testPCIDriver_driver);
	
	//Char Driver Stuff
	printk(KERN_INFO "testPCIDriver module loading... ");

	if (alloc_chrdev_region(&mydev_node, 0, DEVCNT, DEVNAME)) {
		printk(KERN_ERR "alloc_chrdev_region() failed!\n");
		return -1;
	}

	printk(KERN_INFO "Allocated %d devices at major: %d\n", DEVCNT,
	       MAJOR(mydev_node));

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

module_init(testPCIDriver_init_module);

static void __exit testPCIDriver_exit_module(void)
{
	
	cdev_del(&mydev.my_cdev);
	
	pci_unregister_driver(&testPCIDriver_driver);
	
	unregister_chrdev_region(mydev_node, DEVCNT);

	printk(KERN_INFO "testCDriver module unloaded!\n");
}

module_exit(testPCIDriver_exit_module);

static void testPCIDriver_remove(struct pci_dev *pdev)
{

	iounmap(myPCI_dev.hw_addr);
	
	pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	
	printk(KERN_INFO "Successfully unregistered testPCIDriver\n");
}

static ssize_t testPCIDriver_read(struct file *file, char __user *buf, size_t len, loff_t *offset)
{
	int ret; //Get a local kernel buffer sert aside

	/* Make sure our user wasn't bad... */
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}

	mydev.syscall_val = readl(myPCI_dev.hw_addr + LEDOFFSET);
	
	if (copy_to_user(buf, &mydev.syscall_val, sizeof(int))) {
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(int);
	*offset += sizeof(int);

	/* Good to go, so printk the thingy */
	printk(KERN_INFO "User got from us %X\n", mydev.syscall_val);

out:
	return ret;
}

static ssize_t testPCIDriver_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
	/* Have local kernel memory ready */
	int ret;
	/* Make sure our user isn't bad... */
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}

	/*
	Get some memory to copy into...
	kern_buf = kmalloc(len, GFP_KERNEL);

	 ...and make sure it's good to go
	if (!kern_buf) {
		ret = -ENOMEM;
		goto out;
	}
	*/
	/* Copy from the user-provided buffer */
	if (copy_from_user(&mydev.syscall_val, buf, len)) {
		/* uh-oh... */
		ret = -EFAULT;
		goto mem_out;
	}
	writel( *(u32*)&mydev.syscall_val, myPCI_dev.hw_addr  + LEDOFFSET);
	
	ret = len;

	/* print what userspace gave us */
	printk(KERN_INFO "Userspace wrote \"%X\" to us\n", *(u32*)&mydev.syscall_val); //kern_buf);

mem_out:
	//kfree(kern_buf);
out:
	return ret;
}


MODULE_AUTHOR("Devon Mickels");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
