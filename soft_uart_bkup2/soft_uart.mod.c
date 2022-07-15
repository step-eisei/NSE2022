#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x819010fe, "module_layout" },
	{ 0x3ce4ca6f, "disable_irq" },
	{ 0x818d2b0b, "tty_driver_kref_put" },
	{ 0xf9a482f9, "msleep" },
	{ 0x4f097a58, "param_ops_int" },
	{ 0xb4622152, "hrtimer_active" },
	{ 0xc0025372, "hrtimer_forward" },
	{ 0xbe1b7efd, "hrtimer_cancel" },
	{ 0x47229b5c, "gpio_request" },
	{ 0x174a05ca, "gpio_to_desc" },
	{ 0xb43f9365, "ktime_get" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
	{ 0xb1cb409d, "tty_register_driver" },
	{ 0x7e423ba3, "mutex_unlock" },
	{ 0x5b94bd36, "hrtimer_start_range_ns" },
	{ 0xb6fb30e9, "__tty_insert_flip_char" },
	{ 0x8a01a639, "__mutex_init" },
	{ 0xb48e2701, "tty_port_init" },
	{ 0xe8371cdf, "mutex_lock" },
	{ 0xb118f7f1, "gpiod_direction_input" },
	{ 0x745726a2, "gpiod_direction_output_raw" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x2196324, "__aeabi_idiv" },
	{ 0x768229cc, "gpiod_set_debounce" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0x92997ed8, "_printk" },
	{ 0xa36eb6b, "tty_unregister_driver" },
	{ 0x9df65c05, "__tty_alloc_driver" },
	{ 0xfe990052, "gpio_free" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0xfcec0987, "enable_irq" },
	{ 0x570e3b6e, "tty_port_link_device" },
	{ 0x75b0a63e, "gpiod_to_irq" },
	{ 0x3190d729, "gpiod_set_raw_value" },
	{ 0x4848926, "hrtimer_init" },
	{ 0x7a141100, "tty_flip_buffer_push" },
	{ 0x44ca3b4c, "gpiod_get_raw_value" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "E56EB579A6A932A9756DE9C");
