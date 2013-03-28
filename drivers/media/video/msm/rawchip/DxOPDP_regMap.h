/* ============================================================================
*  DxO Labs proprietary and confidential information
*  Copyright (C) DxO Labs 1999-2011 - (All rights reserved)
*  ============================================================================
*
*  The definitions listed below are available in DxOPDP integration guide.
*
*  DxO Labs recommends the customer referring to these definitions before use.
*
*  These values mentioned here are related to a specific customer DxOPDP configuration
*  (RTL parameters and FW capabilities) and delivery.
*
*  It must not be used for any other configuration or delivery.
*
*  ============================================================================ */

#ifndef __DxOPDP_regMap_h__
#define __DxOPDP_regMap_h__

/* ********************************************************************************** */
/* Command registers addresses: */
/* ********************************************************************************** */
#define DxOPDP_boot                                                             0x1a10
#define DxOPDP_execCmd                                                          0x1a08
#define DxOPDP_newFrameCmd                                                      0x1a0c

/* ********************************************************************************** */
/* User parameters registers addresses: */
/* ********************************************************************************** */
#define DxOPDP_ucode_id_7_0                                                     0x0200
#define DxOPDP_ucode_id_15_8                                                    0x0201
#define DxOPDP_hw_id_7_0                                                        0x0202
#define DxOPDP_hw_id_15_8                                                       0x0203
#define DxOPDP_calib_id_0_7_0                                                   0x0204
#define DxOPDP_calib_id_1_7_0                                                   0x0205
#define DxOPDP_calib_id_2_7_0                                                   0x0206
#define DxOPDP_calib_id_3_7_0                                                   0x0207
#define DxOPDP_error_code_7_0                                                   0x0208
#define DxOPDP_visible_line_size_7_0                                            0x0209
#define DxOPDP_visible_line_size_15_8                                           0x020a
#define DxOPDP_mode_7_0                                                         0x020b
#define DxOPDP_image_orientation_7_0                                            0x020c
#define DxOPDP_x_addr_start_7_0                                                 0x020d
#define DxOPDP_x_addr_start_15_8                                                0x020e
#define DxOPDP_y_addr_start_7_0                                                 0x020f
#define DxOPDP_y_addr_start_15_8                                                0x0210
#define DxOPDP_x_addr_end_7_0                                                   0x0211
#define DxOPDP_x_addr_end_15_8                                                  0x0212
#define DxOPDP_y_addr_end_7_0                                                   0x0213
#define DxOPDP_y_addr_end_15_8                                                  0x0214
#define DxOPDP_x_odd_inc_7_0                                                    0x0217
#define DxOPDP_x_odd_inc_15_8                                                   0x0218
#define DxOPDP_y_odd_inc_7_0                                                    0x021b
#define DxOPDP_y_odd_inc_15_8                                                   0x021c
#define DxOPDP_binning_7_0                                                      0x021d
#define DxOPDP_analogue_gain_code_greenr_7_0                                    0x021e
#define DxOPDP_analogue_gain_code_greenr_15_8                                   0x021f
#define DxOPDP_analogue_gain_code_red_7_0                                       0x0220
#define DxOPDP_analogue_gain_code_red_15_8                                      0x0221
#define DxOPDP_analogue_gain_code_blue_7_0                                      0x0222
#define DxOPDP_analogue_gain_code_blue_15_8                                     0x0223
#define DxOPDP_pre_digital_gain_greenr_7_0                                      0x0224
#define DxOPDP_pre_digital_gain_greenr_15_8                                     0x0225
#define DxOPDP_pre_digital_gain_red_7_0                                         0x0226
#define DxOPDP_pre_digital_gain_red_15_8                                        0x0227
#define DxOPDP_pre_digital_gain_blue_7_0                                        0x0228
#define DxOPDP_pre_digital_gain_blue_15_8                                       0x0229
#define DxOPDP_exposure_time_7_0                                                0x022a
#define DxOPDP_exposure_time_15_8                                               0x022b
#define DxOPDP_dead_pixels_correction_lowGain_7_0                               0x022e
#define DxOPDP_dead_pixels_correction_mediumGain_7_0                            0x022f
#define DxOPDP_dead_pixels_correction_strongGain_7_0                            0x0230
#define DxOPDP_frame_number_7_0                                                 0x0231
#define DxOPDP_frame_number_15_8                                                0x0232

/* ********************************************************************************** */
/* User constant definition: */
/* ********************************************************************************** */
#define DxOPDP_execCmd_SettingCmd                                               0x01
#define DxOPDP_mode_features_enabled                                            0x01
#define DxOPDP_mode_black_point_disabled                                        0x08
#define DxOPDP_mode_dead_pixels_disabled                                        0x10
#define DxOPDP_mode_phase_repair_disabled                                       0x20

/* ********************************************************************************** */
/* User parameters default values: */
/* ********************************************************************************** */
#define DxOPDP_dfltVal_ucode_id_7_0                                             0x04
#define DxOPDP_dfltVal_ucode_id_15_8                                            0x01
#define DxOPDP_dfltVal_hw_id_7_0                                                0x5b
#define DxOPDP_dfltVal_hw_id_15_8                                               0xe6
#define DxOPDP_dfltVal_calib_id_0_7_0                                           0x00
#define DxOPDP_dfltVal_calib_id_1_7_0                                           0x00
#define DxOPDP_dfltVal_calib_id_2_7_0                                           0x00
#define DxOPDP_dfltVal_calib_id_3_7_0                                           0x00
#define DxOPDP_dfltVal_error_code_7_0                                           0x00
#define DxOPDP_dfltVal_visible_line_size_7_0                                    0x00
#define DxOPDP_dfltVal_visible_line_size_15_8                                   0x00
#define DxOPDP_dfltVal_mode_7_0                                                 0x00
#define DxOPDP_dfltVal_image_orientation_7_0                                    0x00
#define DxOPDP_dfltVal_x_addr_start_7_0                                         0x00
#define DxOPDP_dfltVal_x_addr_start_15_8                                        0x00
#define DxOPDP_dfltVal_y_addr_start_7_0                                         0x00
#define DxOPDP_dfltVal_y_addr_start_15_8                                        0x00
#define DxOPDP_dfltVal_x_addr_end_7_0                                           0x00
#define DxOPDP_dfltVal_x_addr_end_15_8                                          0x00
#define DxOPDP_dfltVal_y_addr_end_7_0                                           0x00
#define DxOPDP_dfltVal_y_addr_end_15_8                                          0x00
#define DxOPDP_dfltVal_x_odd_inc_7_0                                            0x00
#define DxOPDP_dfltVal_x_odd_inc_15_8                                           0x00
#define DxOPDP_dfltVal_y_odd_inc_7_0                                            0x00
#define DxOPDP_dfltVal_y_odd_inc_15_8                                           0x00
#define DxOPDP_dfltVal_binning_7_0                                              0x00
#define DxOPDP_dfltVal_analogue_gain_code_greenr_7_0                            0x00
#define DxOPDP_dfltVal_analogue_gain_code_greenr_15_8                           0x00
#define DxOPDP_dfltVal_analogue_gain_code_red_7_0                               0x00
#define DxOPDP_dfltVal_analogue_gain_code_red_15_8                              0x00
#define DxOPDP_dfltVal_analogue_gain_code_blue_7_0                              0x00
#define DxOPDP_dfltVal_analogue_gain_code_blue_15_8                             0x00
#define DxOPDP_dfltVal_pre_digital_gain_greenr_7_0                              0x00
#define DxOPDP_dfltVal_pre_digital_gain_greenr_15_8                             0x00
#define DxOPDP_dfltVal_pre_digital_gain_red_7_0                                 0x00
#define DxOPDP_dfltVal_pre_digital_gain_red_15_8                                0x00
#define DxOPDP_dfltVal_pre_digital_gain_blue_7_0                                0x00
#define DxOPDP_dfltVal_pre_digital_gain_blue_15_8                               0x00
#define DxOPDP_dfltVal_exposure_time_7_0                                        0x00
#define DxOPDP_dfltVal_exposure_time_15_8                                       0x00
#define DxOPDP_dfltVal_dead_pixels_correction_lowGain_7_0                       0x80
#define DxOPDP_dfltVal_dead_pixels_correction_mediumGain_7_0                    0x80
#define DxOPDP_dfltVal_dead_pixels_correction_strongGain_7_0                    0x80
#define DxOPDP_dfltVal_frame_number_7_0                                         0xff
#define DxOPDP_dfltVal_frame_number_15_8                                        0xff

/* ********************************************************************************** */
/* Error constant definition: */
/* ********************************************************************************** */
#define DxOPDP_error_code_ok                                                    0x00
#define DxOPDP_error_code_bad_hw_id                                             0x01
#define DxOPDP_error_code_bad_calib_data                                        0x02
#define DxOPDP_error_code_setting_not_ready                                     0x03
#define DxOPDP_error_code_no_matching_setting                                   0x04
#define DxOPDP_error_code_y_addr_end_too_large                                  0x0a
#define DxOPDP_error_code_y_addr_start_odd                                      0x0c
#define DxOPDP_error_code_y_addr_end_even                                       0x0e
#define DxOPDP_error_code_y_boundaries_order                                    0x10
#define DxOPDP_error_code_y_odd_inc_too_large                                   0x12
#define DxOPDP_error_code_y_odd_inc_even                                        0x14
#define DxOPDP_error_code_y_even_inc_even                                       0x18
#define DxOPDP_error_code_invalid_binning                                       0x1c
#define DxOPDP_error_code_invalid_orientation                                   0x1d
#define DxOPDP_error_code_invalid_analogue_gain_code_greenR                     0x1f

#endif /* __DxOPDP_regMap_h__ */
