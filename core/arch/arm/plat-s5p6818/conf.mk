include core/arch/$(ARCH)/plat-$(PLATFORM)/platform_flags.mk

#$(call force,CFG_TEE_CORE_DEBUG,1)

core-platform-cppflags	+= -I$(arch-dir)/include
core-platform-subdirs += \
	$(addprefix $(arch-dir)/, kernel mm tee) $(platform-dir)

$(call force,libutil_with_isoc,y)
$(call force,CFG_GENERIC_BOOT,y)
$(call force,CFG_HWSUPP_MEM_PERM_PXN,y)
$(call force,CFG_S5P6818_UART,y)
$(call force,CFG_S5P6818_TIEOFF,y)
#$(call force,CFG_GIC,y)
$(call force,CFG_PM_STUBS,y)
#$(call force,CFG_SECURE_TIME_SOURCE_CNTPCT,y)
$(call force,CFG_SECURE_TIME_SOURCE_REE,y)
$(call force,CFG_WITH_ARM_TRUSTED_FW,y)
#$(call force,CFG_NO_TA_HASH_SIGN,y)

ifeq ($(CFG_ARM64_core),y)
$(call force,CFG_WITH_LPAE,y)
else
$(call force,CFG_ARM32_core,y)
$(call force,CFG_MMU_V7_TTB,y)
endif

libtomcrypt_with_optimize_size ?= y
CFG_CRYPTO_AES_ARM64_CE ?= $(CFG_ARM64_core)
CFG_CRYPTO_SHA1_ARM32_CE ?= $(CFG_ARM32_core)
CFG_CRYPTO_SHA1_ARM64_CE ?= $(CFG_ARM64_core)
CFG_CRYPTO_SHA256_ARM32_CE ?= $(CFG_ARM32_core)
CFG_CRYPTO_SHA256_ARM64_CE ?= $(CFG_ARM64_core)
CFG_WITH_STACK_CANARIES ?= y

ifeq ($(CFG_CRYPTO_SHA256_ARM32_CE),y)
$(call force,CFG_WITH_VFP,y)
endif
ifeq ($(CFG_CRYPTO_SHA1_ARM32_CE),y)
$(call force,CFG_WITH_VFP,y)
endif
ifeq ($(CFG_CRYPTO_SHA1_ARM64_CE),y)
$(call force,CFG_WITH_VFP,y)
endif
ifeq ($(CFG_CRYPTO_AES_ARM64_CE),y)
$(call force,CFG_WITH_VFP,y)
endif

include mk/config.mk
