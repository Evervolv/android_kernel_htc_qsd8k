obj-y += proc_comm.o
obj-y += io.o timer.o
obj-y += memory.o
ifndef CONFIG_ARCH_MSM8X60
obj-y += dma.o
endif
obj-y += nand_partitions.o
obj-y += pmic.o
obj-y += drv_callback.o
obj-y += htc_board_tags.o

obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock-arm11.o idle.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += idle-v7.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += arch-init-scorpion.o

obj-$(CONFIG_ARCH_QSD8X50) += acpuclock-qsd8x50.o
obj-$(CONFIG_ARCH_MSM7X30) += acpuclock-7x30.o

ifndef CONFIG_ARCH_MSM8X60
obj-y += irq.o
endif

obj-$(CONFIG_ARCH_MSM8X60) += clock-dummy.o iommu.o iommu_dev.o devices-msm8x60-iommu.o
obj-$(CONFIG_MSM_PROC_COMM) += proc_comm.o clock-pcom.o vreg.o
obj-$(CONFIG_MSM_PROC_COMM) += clock.o
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o
obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
obj-$(CONFIG_MSM_SCM) += scm.o scm-boot.o

obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SMP) += headsmp.o platsmp.o

obj-$(CONFIG_MSM_DAL) += dal.o

obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_QDSP5V2) += qdsp5v2/
obj-$(CONFIG_MSM_QDSP6) += qdsp6/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o
obj-$(CONFIG_MSM_REMOTE_SPINLOCK) += remote_spinlock.o
obj-$(CONFIG_MSM_SSBI) += ssbi.o

obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MACH_TROUT) += board-trout-wifi.o
obj-$(CONFIG_MACH_TROUT) += devices_htc.o
obj-$(CONFIG_TROUT_BATTCHG) += htc_battery_trout.o

obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire.o board-sapphire-gpio.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-keypad.o board-sapphire-panel.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-mmc.o board-sapphire-wifi.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-rfkill.o msm_vibrator.o
obj-$(CONFIG_MACH_SAPPHIRE) += devices_htc.o
obj-$(CONFIG_MACH_SAPPHIRE) += htc_akm_cal.o htc_wifi_nvs.o

obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi.o board-mahimahi-panel.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-keypad.o board-mahimahi-mmc.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-rfkill.o htc_wifi_nvs.o htc_awb_cal.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-wifi.o board-mahimahi-audio.o
obj-$(CONFIG_MACH_MAHIMAHI) += msm_vibrator.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-microp.o

obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o devices-msm7x00.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut-keypad.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut-panel.o fish_battery.o

obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-keypad.o fish_battery.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-panel.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-mmc.o

obj-$(CONFIG_ARCH_MSM7X30) += devices-msm7x30.o
obj-$(CONFIG_ARCH_QSD8X50) += devices-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_SURF) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50A_ST1_5) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_FFA) += board-qsd8x50.o
obj-$(CONFIG_ARCH_MSM8X60) += board-msm8x60.o

obj-$(CONFIG_MACH_MSM7X30_SURF) += board-msm7x30.o
obj-$(CONFIG_MACH_MSM7X30_SURF) += board-msm7x30-panel.o
obj-$(CONFIG_MACH_MSM7X30_SURF) += board-msm7x30-audio.o

obj-$(CONFIG_ARCH_MSM7X30) += gpiomux-7x30.o gpiomux-v1.o gpiomux.o
obj-$(CONFIG_ARCH_QSD8X50) += gpiomux-8x50.o gpiomux-v1.o gpiomux.o
obj-$(CONFIG_ARCH_MSM8X60) += gpiomux-8x60.o gpiomux-v2.o gpiomux.o
ifdef CONFIG_MSM_V2_TLMM
obj-y	+= gpio-v2.o
else
obj-y	+= gpio.o
endif

obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic.o board-supersonic-panel.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-keypad.o board-supersonic-mmc.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-rfkill.o board-supersonic-audio.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_SUPERSONIC) += msm_vibrator.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-microp.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-tpa2018d1.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-tpa6130.o
obj-$(CONFIG_MACH_SUPERSONIC) += htc_wifi_nvs.o htc_bluetooth.o
obj-$(CONFIG_MACH_SUPERSONIC) += htc_acoustic_qsd.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-flashlight.o

obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o
obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_HTC_PWRSPLY) += htc_power_supply.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
obj-$(CONFIG_HTC_35MM_JACK) += htc_35mm_jack.o

obj-$(CONFIG_MICROP_COMMON) += atmega_microp_common.o
obj-$(CONFIG_HTC_HEADSET_MGR) += htc_headset_mgr.o
obj-$(CONFIG_HTC_HEADSET_GPIO) += htc_headset_gpio.o
obj-$(CONFIG_HTC_HEADSET_MICROP) += htc_headset_microp.o
obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_PERFLOCK) += htc_set_perflock.o

obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-tpa2018d1.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-smb329.o

obj-$(CONFIG_ARCH_MSM_FLASHLIGHT) += msm_flashlight.o

