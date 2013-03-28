#ifndef HIMAX8526a_H
#define HIMAX8526a_H
#include <linux/types.h>
#include <linux/i2c.h>

#define HIMAX8526A_NAME "Himax8526a"
#define HIMAX8526A_FINGER_SUPPORT_NUM 4
struct himax_config_init_api {
	int (*i2c_himax_master_write)(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t retry);
	int (*i2c_himax_write_command)(struct i2c_client *client, uint8_t command, uint8_t retry);
	int (*i2c_himax_read_command)(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t retry);
};

struct himax_i2c_platform_data_config_type_1 {
	uint8_t version;
	uint8_t tw_id;
	uint8_t c1[3];
	uint8_t c2[3];
	uint8_t c3[5];
	uint8_t c4[2];
	uint8_t c5[2];
	uint8_t c6[2];
	uint8_t c7[3];
	uint8_t c8[2];
	uint8_t c9[4];
	uint8_t c10[6];
	uint8_t c11[15];
	uint8_t c12[5];
	uint8_t c13[8];
	uint8_t c14[4];
	uint8_t c15[11];
	uint8_t c16[4];
	uint8_t c17[2];
	uint8_t c18[11];
	uint8_t c19[11];
	uint8_t c20[11];
	uint8_t c21[11];
	uint8_t c22[11];
	uint8_t c23[11];
	uint8_t c24[11];
	uint8_t c25[11];
	uint8_t c26[11];
	uint8_t c27[11];
	uint8_t c28[11];
	uint8_t c29[11];
	uint8_t c30[33];
	uint8_t c31[49];
	uint8_t c32[10];
	uint8_t c33[2];
	uint8_t c34[5];
	uint8_t c35[5];
	uint8_t c36[8];
	uint8_t c37[3];
	uint8_t c38[2];
	uint8_t c39[5];
	uint8_t c40[5];
	uint8_t c41[2];
	uint8_t c42[5];
	uint8_t c43[6];
	uint8_t c44[6];
	uint8_t checksum[3];
};

struct himax_i2c_platform_data_config_type_2 {
	uint8_t version;
	uint8_t tw_id;
	uint8_t c1[3];
	uint8_t c2[3];
	uint8_t c3[5];
	uint8_t c4[2];
	uint8_t c5[2];
	uint8_t c6[2];
	uint8_t c7[3];
	uint8_t c8[2];
	uint8_t c9[4];
	uint8_t c10[6];
	uint8_t c11[15];
	uint8_t c12[5];
	uint8_t c13[8];
	uint8_t c14[4];
	uint8_t c15[11];
	uint8_t c16[4];
	uint8_t c17[2];
	uint8_t c18[11];
	uint8_t c19[11];
	uint8_t c20[11];
	uint8_t c21[11];
	uint8_t c22[11];
	uint8_t c23[11];
	uint8_t c24[11];
	uint8_t c25[11];
	uint8_t c26[11];
	uint8_t c27[11];
	uint8_t c28[11];
	uint8_t c29[11];
	uint8_t c30[44];
	uint8_t c31[69];
	uint8_t c32[11];
	uint8_t c33[2];
	uint8_t c34[5];
	uint8_t c35[5];
	uint8_t c36[8];
	uint8_t c37[3];
	uint8_t c38[2];
	uint8_t c39[5];
	uint8_t c40[5];
	uint8_t c41[2];
	uint8_t c42[5];
	uint8_t c43[6];
	uint8_t c44[6];
	uint8_t c45[3];
	uint8_t checksum[3];
};

struct himax_i2c_platform_data_config_type_3 {
	uint8_t version;
	uint8_t tw_id;
	uint8_t c1[11];
	uint8_t c2[11];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[65];
	uint8_t c14[97];
	uint8_t c15[8];
	uint8_t c16[4];
	uint8_t c17[6];
	uint8_t c18[15];
	uint8_t c19[17];
	uint8_t c20[3];
	uint8_t c21[5];
	uint8_t c22[5];
	uint8_t c23[10];
	uint8_t c24[5];
	uint8_t c25[2];
	uint8_t c26[3];
	uint8_t c27[2];
	uint8_t c28[5];
	uint8_t c29[5];
	uint8_t c30[2];
	uint8_t c31[5];
	uint8_t c32[6];
	uint8_t c33[6];
	uint8_t c34[3];
	uint8_t c35[4];
	uint8_t c36[5];
	uint8_t c37[2];
	uint8_t c38[2];
	uint8_t c39[2];
	uint8_t c40[2];
	uint8_t c41[3];
	uint8_t c42[2];
	uint8_t c43[4];
	uint8_t c44[3];
	uint8_t c45[33];
	uint8_t c46[17];
	uint8_t c47[5];
	uint8_t c48[11];
	uint8_t c49[4];
	uint8_t c50[3];
	uint8_t checksum[3];
};

struct himax_i2c_platform_data {
	/* common variables */
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_width_min;
	int abs_width_max;
	uint8_t powerOff3V3;
	int (*power)(int on);
	int gpio_irq;
	uint8_t slave_addr;
	uint32_t event_htc_enable;
	uint8_t cable_config[2];

	/* To support Sprint 2a2b request, inject 2a2b in google format. Naming as synaptic solution*/
	uint8_t support_htc_event;
	/* Support Sprint 2a2b --End--*/

	/* To decide Protocol A+ID(0) or B+ID(1) */
	uint8_t protocol_type;

	/* For fake event --start-- */
	int screenWidth;
	int screenHeight;
	/* For fake event --end-- */

	void (*reset)(void);
	int (*loadSensorConfig)(struct i2c_client *client, struct himax_i2c_platform_data *pdata, struct himax_config_init_api *i2c_api);
	/* for compatible and caching purpose */
	uint8_t version;
	uint8_t tw_id;
	/* for resume ESD recovery clock divider restore */
	uint8_t *regCD;
	/* types of configurations */
	struct himax_i2c_platform_data_config_type_1 *type1;
	int type1_size;
	struct himax_i2c_platform_data_config_type_2 *type2;
	int type2_size;
	struct himax_i2c_platform_data_config_type_3 *type3;
	int type3_size;
};

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};

#endif

