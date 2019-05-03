ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
obj-$(CONFIG_MSMB_CAMERA) += msm_led_flash.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_i2c_trigger.o
# [5830][FlashLED][akenhsu] Porting LM3644TT Flash LED for ASUS 5816 20160204 BEGIN
# obj-$(CONFIG_MSMB_CAMERA) += adp1660.o
# obj-$(CONFIG_MSMB_CAMERA) += bd7710.o
# [5830][FlashLED]20160204 END
obj-$(CONFIG_MSMB_CAMERA) += msm_led_torch.o
obj-$(CONFIG_MSMB_CAMERA) += msm_flash.o
# [5830][FlashLED][akenhsu] Porting LM3644TT Flash LED for ASUS 5816 20160204 BEGIN
# obj-$(CONFIG_MSMB_CAMERA) += lm3642.o
ifeq (${CONFIG_BSP_HW_SKU_5830}, 1)
$(warning CONFIG_BSP_HW_SKU_5830 -> lm3644tt)
obj-$(CONFIG_MSMB_CAMERA) += lm3644tt.o
endif
# [5830][FlashLED]20160204 END

# [5833][FlashLED][akenhsu] Porting LM3648 Flash LED for ASUS 5833 20160324 BEGIN
ifeq (${CONFIG_BSP_HW_SKU_5833}, 1)
$(warning CONFIG_BSP_HW_SKU_5833 -> lm3648)
obj-$(CONFIG_MSMB_CAMERA) += lm3648.o
endif
# [5833][FlashLED][akenhsu] 20160324 END
