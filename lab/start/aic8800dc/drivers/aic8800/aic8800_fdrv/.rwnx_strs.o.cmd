cmd_/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.o := gcc -Wp,-MMD,/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/.rwnx_strs.o.d  -nostdinc -isystem /usr/lib/gcc/aarch64-linux-gnu/9/include -I./arch/arm64/include -I./arch/arm64/include/generated  -I./include -I./arch/arm64/include/uapi -I./arch/arm64/include/generated/uapi -I./include/uapi -I./include/generated/uapi -include ./include/linux/kconfig.h -include ./include/linux/compiler_types.h -D__KERNEL__ -mlittle-endian -DKASAN_SHADOW_SCALE_SHIFT=3 -fmacro-prefix-map=./= -Wall -Wundef -Werror=strict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -fshort-wchar -fno-PIE -Werror=implicit-function-declaration -Werror=implicit-int -Werror=return-type -Wno-format-security -std=gnu89 -mgeneral-regs-only -DCONFIG_CC_HAS_K_CONSTRAINT=1 -Wno-psabi -mabi=lp64 -fno-asynchronous-unwind-tables -fno-unwind-tables -mbranch-protection=none -DKASAN_SHADOW_SCALE_SHIFT=3 -fno-delete-null-pointer-checks -Wno-frame-address -Wno-format-truncation -Wno-format-overflow -Wno-address-of-packed-member -O2 --param=allow-store-data-races=0 -Wframe-larger-than=4096 -fstack-protector-strong -Wimplicit-fallthrough -Wno-unused-but-set-variable -Wno-unused-const-variable -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -pg -Wdeclaration-after-statement -Wvla -Wno-pointer-sign -Wno-stringop-truncation -Wno-array-bounds -Wno-stringop-overflow -Wno-restrict -Wno-maybe-uninitialized -fno-strict-overflow -fno-stack-check -fconserve-stack -Werror=date-time -Werror=incompatible-pointer-types -Werror=designated-init -Wno-packed-not-aligned -mstack-protector-guard=sysreg -mstack-protector-guard-reg=sp_el0 -mstack-protector-guard-offset=1392 -DCONFIG_RWNX_P2P_DEBUGFS -DNX_VIRT_DEV_MAX=4 -DNX_REMOTE_STA_MAX_FOR_OLD_IC=8 -DNX_REMOTE_STA_MAX=32 -DNX_MU_GROUP_MAX=62 -DNX_TXDESC_CNT=64 -DNX_TX_MAX_RATES=4 -DNX_CHAN_CTXT_CNT=3 -DCONFIG_START_FROM_BOOTROM -DCONFIG_VRF_DCDC_MODE -DCONFIG_ROM_PATCH_EN -DCONFIG_COEX -DCONFIG_RWNX_FULLMAC -I/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/. -DCONFIG_RWNX_RADAR -DCONFIG_RWNX_DBG -DCONFIG_RFTEST -DAICWF_USB_SUPPORT -DCONFIG_USER_MAX=1 -DNX_TXQ_CNT=5 -DAICWF_RX_REORDER -DAICWF_ARP_OFFLOAD -DUSE_5G -DCONFIG_USB_BT -DCONFIG_ALIGN_8BYTES -DCONFIG_TXRX_THREAD_PRIO -DCONFIG_USB_ALIGN_DATA -DCONFIG_MAC_RANDOM_IF_NO_MAC_IN_EFUSE -DDEFAULT_COUNTRY_CODE=""\"00""\" -DCONFIG_RX_NETIF_RECV_SKB -DCONFIG_USB_MSG_OUT_EP -DCONFIG_USB_MSG_IN_EP -DCONFIG_USE_USB_ZERO_PACKET -DCONFIG_STA_SCAN_WHEN_P2P_WORKING -DCONFIG_SUPPORT_REALTIME_CHANGE_MAC -DCONFIG_PREALLOC_TXQ -DCONFIG_DPD -DCONFIG_FORCE_DPD_CALIB -DCONFIG_DPD -Wno-implicit-fallthrough -Wno-attribute-warning -Wno-discarded-qualifiers -Wno-frame-larger-than -I/usr/src/linux-headers-5.10.104-tegra-ubuntu20.04_aarch64/nvidia/include -I/usr/src/linux-headers-5.10.104-tegra-ubuntu20.04_aarch64/nvlink/include -I/usr/src/linux-headers-5.10.104-tegra-ubuntu20.04_aarch64/nvgpu/include -I/usr/src/linux-headers-5.10.104-tegra-ubuntu20.04_aarch64/nvgpu-next/include -I/usr/src/linux-headers-5.10.104-tegra-ubuntu20.04_aarch64/nvidia-t239/include  -DMODULE  -DKBUILD_BASENAME='"rwnx_strs"' -DKBUILD_MODNAME='"aic8800_fdrv"' -c -o /home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.o /home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.c

source_/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.o := /home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.c

deps_/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.o := \
    $(wildcard include/config/rwnx/fullmac.h) \
  include/linux/kconfig.h \
    $(wildcard include/config/cc/version/text.h) \
    $(wildcard include/config/cpu/big/endian.h) \
    $(wildcard include/config/booger.h) \
    $(wildcard include/config/foo.h) \
  include/linux/compiler_types.h \
    $(wildcard include/config/have/arch/compiler/h.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/cc/has/asm/inline.h) \
  include/linux/compiler_attributes.h \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arm64.h) \
    $(wildcard include/config/retpoline.h) \
    $(wildcard include/config/arch/use/builtin/bswap.h) \
    $(wildcard include/config/kcov.h) \
  arch/arm64/include/asm/compiler.h \
  /home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/lmac_msg.h \
    $(wildcard include/config/rwnx/fhost.h) \
    $(wildcard include/config/usb/bt.h) \
  /home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/lmac_mac.h \
    $(wildcard include/config/he/for/old/kernel.h) \
    $(wildcard include/config/vht/for/old/kernel.h) \
  /home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/lmac_types.h \
    $(wildcard include/config/rwnx/tl4.h) \
  include/generated/uapi/linux/version.h \
  include/linux/types.h \
    $(wildcard include/config/have/uid16.h) \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/arch/dma/addr/t/64bit.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  include/uapi/linux/types.h \
  arch/arm64/include/generated/uapi/asm/types.h \
  include/uapi/asm-generic/types.h \
  include/asm-generic/int-ll64.h \
  include/uapi/asm-generic/int-ll64.h \
  arch/arm64/include/uapi/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/uapi/asm-generic/bitsperlong.h \
  include/uapi/linux/posix_types.h \
  include/linux/stddef.h \
  include/uapi/linux/stddef.h \
  include/linux/compiler_types.h \
  arch/arm64/include/uapi/asm/posix_types.h \
  include/uapi/asm-generic/posix_types.h \
  include/linux/bits.h \
  include/linux/const.h \
  include/vdso/const.h \
  include/uapi/linux/const.h \
  include/vdso/bits.h \
  include/linux/build_bug.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/stack/validation.h) \
  arch/arm64/include/generated/asm/rwonce.h \
  include/asm-generic/rwonce.h \
  include/linux/kasan-checks.h \
    $(wildcard include/config/kasan.h) \
  include/linux/kcsan-checks.h \
    $(wildcard include/config/kcsan.h) \
    $(wildcard include/config/kcsan/ignore/atomics.h) \

/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.o: $(deps_/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.o)

$(deps_/home/unitree/lab/start/aic8800dc/drivers/aic8800/aic8800_fdrv/rwnx_strs.o):
