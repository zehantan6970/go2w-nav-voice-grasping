#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

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
	{ 0xd31aec96, "module_layout" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x52d1fb52, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf9a482f9, "msleep" },
	{ 0x1fdc7df2, "_mcount" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0x2eac42c4, "param_ops_int" },
	{ 0x98cf60b3, "strlen" },
	{ 0xd376c702, "send_sig" },
	{ 0xc3690fc, "_raw_spin_lock_bh" },
	{ 0x56470118, "__warn_printk" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x759ea8c3, "usb_kill_urb" },
	{ 0x7ae64fcf, "filp_close" },
	{ 0x726bc3c7, "wait_for_completion_killable_timeout" },
	{ 0x5f89ca6b, "__dev_kfree_skb_any" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x3096be16, "names_cachep" },
	{ 0x999e8297, "vfree" },
	{ 0x9ba8148b, "kthread_create_on_node" },
	{ 0xd9a15b9e, "skb_unlink" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x2fec1e0c, "param_ops_string" },
	{ 0x6033551a, "usb_unanchor_urb" },
	{ 0x93d6dd8c, "complete_all" },
	{ 0x4a5a32c1, "__netdev_alloc_skb" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xcd23c26, "skb_dequeue_tail" },
	{ 0x7bcc6d6f, "kernel_read" },
	{ 0xdcb764ad, "memset" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xbf2d5410, "usb_deregister" },
	{ 0xc5850110, "printk" },
	{ 0x5ec05146, "kthread_stop" },
	{ 0x2e3bcce2, "wait_for_completion_interruptible" },
	{ 0x24c1f014, "kmem_cache_free" },
	{ 0xcc3f5490, "skb_pull" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x1ca7fb0d, "usb_submit_urb" },
	{ 0xc6823796, "kmem_cache_alloc" },
	{ 0xa916b694, "strnlen" },
	{ 0x962c8ae1, "usb_kill_anchored_urbs" },
	{ 0xe46021ca, "_raw_spin_unlock_bh" },
	{ 0x86332725, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x1000e51, "schedule" },
	{ 0xf424b0fe, "cpu_hwcap_keys" },
	{ 0x6a997b41, "wake_up_process" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x9ac23f86, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0x6df1aaf1, "kernel_sigaction" },
	{ 0x571fd711, "usb_register_driver" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xa6257a2f, "complete" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xef66446f, "consume_skb" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0x98d7354b, "skb_put" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xcb855872, "usb_free_urb" },
	{ 0x826f1e2d, "usb_anchor_urb" },
	{ 0x655fc02e, "usb_alloc_urb" },
	{ 0xda05b7b8, "filp_open" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:vA69Cp8800d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:vA69Cp8801d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:vA69Cp8D80d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:vA69Cp8D81d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "C2D9285D533322FE63337AA");
