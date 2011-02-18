obj-y += io.o dma.o memory.o timer.o
obj-y += clock.o
obj-$(CONFIG_CPU_FREQ_MSM) += cpufreq.o
obj-y += socinfo.o

obj-$(CONFIG_MSM_PROC_COMM) += proc_comm.o
ifndef CONFIG_ARCH_MSM8X60
	obj-$(CONFIG_MSM_PROC_COMM) += clock-pcom.o
	obj-$(CONFIG_MSM_PROC_COMM) += vreg.o
	ifdef CONFIG_MSM_PROC_COMM
ifndef CONFIG_ARCH_FSM9XXX
		obj-$(CONFIG_REGULATOR) += footswitch-pcom.o
endif
	endif
endif

obj-y += drv_callback.o
obj-y += htc_board_tags.o

obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock-arm11.o idle.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += arch-init-scorpion.o

obj-$(CONFIG_ARCH_QSD8X50) += acpuclock-qsd8x50.o
obj-$(CONFIG_ARCH_MSM7X30) += acpuclock-7x30.o
obj-$(CONFIG_MSM_CPU_AVS) += avs.o
obj-$(CONFIG_MSM_AVS_HW) += avs_hw.o

ifndef CONFIG_ARCH_MSM8X60
obj-y += irq.o
endif

obj-$(CONFIG_ARCH_MSM8X60) += clock-dummy.o iommu.o iommu_dev.o devices-msm8x60-iommu.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SMP) += headsmp.o platsmp.o
obj-$(CONFIG_CPU_V7) += idle-v7.o
obj-$(CONFIG_MSM_SCM) += scm.o scm-boot.o
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o
obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MSM_SMD_LOGGING) += smem_log.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_SMD) += last_radio_log.o

ifndef CONFIG_ARCH_MSM9615
ifndef CONFIG_ARCH_MSM8960
ifndef CONFIG_ARCH_MSM8X60
	obj-$(CONFIG_MSM_SMD) += pmic.o
endif
endif
endif
ifndef CONFIG_ARCH_MSM8960
ifndef CONFIG_ARCH_MSM8X60
ifndef CONFIG_ARCH_APQ8064
	obj-y += nand_partitions.o
endif
endif
endif

obj-$(CONFIG_MSM_DAL) += dal.o

obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_QDSP5V2) += qdsp5v2/
obj-$(CONFIG_MSM_QDSP6) += qdsp6/
obj-$(CONFIG_MSM_REMOTE_SPINLOCK) += remote_spinlock.o
obj-$(CONFIG_MSM_SSBI) += ssbi.o
ifdef CONFIG_PM
	obj-y += pm.o
else
	obj-y += no-pm.o
endif

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
obj-$(CONFIG_MACH_MAHIMAHI) += htc_acoustic_qsd.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-tpa2018d1.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-smb329.o

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

obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec.o board-incrediblec-panel.o board-incrediblec-tv.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-keypad.o board-incrediblec-mmc.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-rfkill.o htc_wifi_nvs.o  board-incrediblec-audio.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-microp.o htc_bluetooth.o
obj-$(CONFIG_MACH_INCREDIBLEC) += msm_vibrator.o
obj-$(CONFIG_MACH_INCREDIBLEC) += proc_engineerid.o board-incrediblec-tpa6130.o
obj-$(CONFIG_MACH_INCREDIBLEC) += htc_acoustic_qsd.o

obj-$(CONFIG_MACH_BRAVO) += board-bravo.o board-bravo-panel.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-keypad.o board-bravo-mmc.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-rfkill.o htc_wifi_nvs.o htc_awb_cal.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-wifi.o board-bravo-audio.o
obj-$(CONFIG_MACH_BRAVO) += msm_vibrator.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-microp.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-tpa2018d1.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-smb329.o

obj-$(CONFIG_MACH_BRAVOC) += board-bravo.o board-bravo-panel.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravo-keypad.o board-bravo-mmc.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravo-rfkill.o htc_wifi_nvs.o htc_awb_cal.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravo-wifi.o board-bravo-audio.o
obj-$(CONFIG_MACH_BRAVOC) += msm_vibrator.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravo-microp.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravo-tpa2018d1.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravo-smb329.o

obj-$(CONFIG_HTC_ACOUSTIC) += htc_acoustic.o
obj-$(CONFIG_HTC_ACOUSTIC_QSD) += htc_acoustic_qsd.o
obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o
obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_HTC_PWRSPLY) += htc_power_supply.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
obj-$(CONFIG_HTC_35MM_JACK) += htc_35mm_jack.o
obj-$(CONFIG_HTC_HEADSET_MGR) += htc_headset_mgr.o
obj-$(CONFIG_HTC_HEADSET_GPIO) += htc_headset_gpio.o
obj-$(CONFIG_HTC_HEADSET_MICROP) += htc_headset_microp.o
obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_PERFLOCK) += htc_set_perflock.o
obj-$(CONFIG_MICROP_COMMON) += atmega_microp_common.o
obj-$(CONFIG_ARCH_MSM_FLASHLIGHT) += msm_flashlight.o
