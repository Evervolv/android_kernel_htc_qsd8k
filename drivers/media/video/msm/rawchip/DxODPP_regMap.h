/* ============================================================================
*  DxO Labs proprietary and confidential information
*  Copyright (C) DxO Labs 1999-2011 - (All rights reserved)
*  ============================================================================
*
*  The definitions listed below are available in DxODPP integration guide.
*
*  DxO Labs recommends the customer referring to these definitions before use.
*
*  These values mentioned here are related to a specific customer DxODPP configuration
*  (RTL parameters and FW capabilities) and delivery.
*
*  It must not be used for any other configuration or delivery.
*
*  ============================================================================ */

#ifndef __DxODPP_regMap_h__
#define __DxODPP_regMap_h__

/* ********************************************************************************** */
/* Command registers addresses: */
/* ********************************************************************************** */
#define DxODPP_boot                                                             0xd010
#define DxODPP_execCmd                                                          0xd008
#define DxODPP_newFrameCmd                                                      0xd00c

/* ********************************************************************************** */
/* User parameters registers addresses: */
/* ********************************************************************************** */
#define DxODPP_ucode_id_7_0                                                     0x0200
#define DxODPP_ucode_id_15_8                                                    0x0201
#define DxODPP_hw_id_7_0                                                        0x0202
#define DxODPP_hw_id_15_8                                                       0x0203
#define DxODPP_calib_id_0_7_0                                                   0x0204
#define DxODPP_calib_id_1_7_0                                                   0x0205
#define DxODPP_calib_id_2_7_0                                                   0x0206
#define DxODPP_calib_id_3_7_0                                                   0x0207
#define DxODPP_error_code_7_0                                                   0x0208
#define DxODPP_visible_line_size_7_0                                            0x0209
#define DxODPP_visible_line_size_15_8                                           0x020a
#define DxODPP_mode_7_0                                                         0x020b
#define DxODPP_image_orientation_7_0                                            0x020c
#define DxODPP_x_addr_start_7_0                                                 0x020d
#define DxODPP_x_addr_start_15_8                                                0x020e
#define DxODPP_y_addr_start_7_0                                                 0x020f
#define DxODPP_y_addr_start_15_8                                                0x0210
#define DxODPP_x_addr_end_7_0                                                   0x0211
#define DxODPP_x_addr_end_15_8                                                  0x0212
#define DxODPP_y_addr_end_7_0                                                   0x0213
#define DxODPP_y_addr_end_15_8                                                  0x0214
#define DxODPP_x_even_inc_7_0                                                   0x0215
#define DxODPP_x_even_inc_15_8                                                  0x0216
#define DxODPP_x_odd_inc_7_0                                                    0x0217
#define DxODPP_x_odd_inc_15_8                                                   0x0218
#define DxODPP_y_even_inc_7_0                                                   0x0219
#define DxODPP_y_even_inc_15_8                                                  0x021a
#define DxODPP_y_odd_inc_7_0                                                    0x021b
#define DxODPP_y_odd_inc_15_8                                                   0x021c
#define DxODPP_analogue_gain_code_greenr_7_0                                    0x021e
#define DxODPP_analogue_gain_code_greenr_15_8                                   0x021f
#define DxODPP_pre_digital_gain_greenr_7_0                                      0x0224
#define DxODPP_pre_digital_gain_greenr_15_8                                     0x0225
#define DxODPP_exposure_time_7_0                                                0x022a
#define DxODPP_exposure_time_15_8                                               0x022b
#define DxODPP_temporal_smoothing_7_0                                           0x022e
#define DxODPP_flash_preflash_ratio_7_0                                         0x022f
#define DxODPP_flash_preflash_ratio_15_8                                        0x0230
#define DxODPP_focal_info_7_0                                                   0x0231
#define DxODPP_frame_number_7_0                                                 0x0232
#define DxODPP_frame_number_15_8                                                0x0233
#define DxODPP_last_estimation_frame_number_7_0                                 0x0234
#define DxODPP_last_estimation_frame_number_15_8                                0x0235

/* ********************************************************************************** */
/* User constant definition: */
/* ********************************************************************************** */
#define DxODPP_execCmd_SettingCmd                                               0x01
#define DxODPP_mode_cls_msk                                                     0x01
#define DxODPP_mode_grGb_msk                                                    0x02
#define DxODPP_mode_unused_bit5                                                 0x20
#define DxODPP_mode_unused_bit6                                                 0x40
#define DxODPP_mode_preFlash                                                    0x08
#define DxODPP_mode_flash                                                       0x10
#define DxODPP_mode_restartEstim                                                0x80

/* ********************************************************************************** */
/* User parameters default values: */
/* ********************************************************************************** */
#define DxODPP_dfltVal_ucode_id_7_0                                             0x04
#define DxODPP_dfltVal_ucode_id_15_8                                            0x01
#define DxODPP_dfltVal_hw_id_7_0                                                0xe8
#define DxODPP_dfltVal_hw_id_15_8                                               0xeb
#define DxODPP_dfltVal_calib_id_0_7_0                                           0x00
#define DxODPP_dfltVal_calib_id_1_7_0                                           0x00
#define DxODPP_dfltVal_calib_id_2_7_0                                           0x00
#define DxODPP_dfltVal_calib_id_3_7_0                                           0x00
#define DxODPP_dfltVal_error_code_7_0                                           0x00
#define DxODPP_dfltVal_visible_line_size_7_0                                    0x00
#define DxODPP_dfltVal_visible_line_size_15_8                                   0x00
#define DxODPP_dfltVal_mode_7_0                                                 0x00
#define DxODPP_dfltVal_image_orientation_7_0                                    0x00
#define DxODPP_dfltVal_x_addr_start_7_0                                         0x00
#define DxODPP_dfltVal_x_addr_start_15_8                                        0x00
#define DxODPP_dfltVal_y_addr_start_7_0                                         0x00
#define DxODPP_dfltVal_y_addr_start_15_8                                        0x00
#define DxODPP_dfltVal_x_addr_end_7_0                                           0x00
#define DxODPP_dfltVal_x_addr_end_15_8                                          0x00
#define DxODPP_dfltVal_y_addr_end_7_0                                           0x00
#define DxODPP_dfltVal_y_addr_end_15_8                                          0x00
#define DxODPP_dfltVal_x_even_inc_7_0                                           0x00
#define DxODPP_dfltVal_x_even_inc_15_8                                          0x00
#define DxODPP_dfltVal_x_odd_inc_7_0                                            0x00
#define DxODPP_dfltVal_x_odd_inc_15_8                                           0x00
#define DxODPP_dfltVal_y_even_inc_7_0                                           0x00
#define DxODPP_dfltVal_y_even_inc_15_8                                          0x00
#define DxODPP_dfltVal_y_odd_inc_7_0                                            0x00
#define DxODPP_dfltVal_y_odd_inc_15_8                                           0x00
#define DxODPP_dfltVal_analogue_gain_code_greenr_7_0                            0x00
#define DxODPP_dfltVal_analogue_gain_code_greenr_15_8                           0x00
#define DxODPP_dfltVal_pre_digital_gain_greenr_7_0                              0x00
#define DxODPP_dfltVal_pre_digital_gain_greenr_15_8                             0x00
#define DxODPP_dfltVal_exposure_time_7_0                                        0x00
#define DxODPP_dfltVal_exposure_time_15_8                                       0x00
#define DxODPP_dfltVal_temporal_smoothing_7_0                                   0x00
#define DxODPP_dfltVal_flash_preflash_ratio_7_0                                 0x00
#define DxODPP_dfltVal_flash_preflash_ratio_15_8                                0x00
#define DxODPP_dfltVal_focal_info_7_0                                           0x00
#define DxODPP_dfltVal_frame_number_7_0                                         0xff
#define DxODPP_dfltVal_frame_number_15_8                                        0xff
#define DxODPP_dfltVal_last_estimation_frame_number_7_0                         0xff
#define DxODPP_dfltVal_last_estimation_frame_number_15_8                        0xff

/* ********************************************************************************** */
/* Error constant definition: */
/* ********************************************************************************** */
#define DxODPP_error_code_ok                                                    0x00
#define DxODPP_error_code_bad_hw_id                                             0x01
#define DxODPP_error_code_bad_calib_data                                        0x02
#define DxODPP_error_code_setting_not_ready                                     0x03
#define DxODPP_error_code_no_matching_setting                                   0x04
#define DxODPP_error_code_invalid_cmd                                           0x05
#define DxODPP_error_code_invalid_mode                                          0x06
#define DxODPP_error_code_y_addr_start_too_large                                0x08
#define DxODPP_error_code_y_addr_end_too_large                                  0x0a
#define DxODPP_error_code_y_addr_start_odd                                      0x0c
#define DxODPP_error_code_y_addr_end_even                                       0x0e
#define DxODPP_error_code_y_boundaries_order                                    0x10
#define DxODPP_error_code_y_odd_inc_too_large                                   0x12
#define DxODPP_error_code_x_decim_unsupported                                   0x15
#define DxODPP_error_code_y_decim_unsupported                                   0x16
#define DxODPP_error_code_y_even_inc_too_large                                  0x1a
#define DxODPP_error_code_temporal_smoothing_too_large                          0x1b

#endif /* __DxODPP_regMap_h__ */
