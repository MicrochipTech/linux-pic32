/*
 * ak4953a.c  --  audio driver for AK4953A
 *
 *  Joshua Henderson <joshua.henderson@microchip.com>
 *  Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
 *
 * Copyright (C) 2013 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      13/05/27	    2.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>

#include "ak4953a.h"

/* ak4953a default register settings */
static const struct reg_default ak4953a_reg_defaults[] = {
	{ 0x00, 0x00 },	/* 0x00	AK4953A_00_POWER_MANAGEMENT1	*/
	{ 0x01, 0x00 },	/* 0x01	AK4953A_01_POWER_MANAGEMENT2	*/
	{ 0x02, 0x03 },	/* 0x02	AK4953A_02_SIGNAL_SELECT1	*/
	{ 0x03, 0x00 },	/* 0x03	AK4953A_03_SIGNAL_SELECT2	*/
	{ 0x04, 0x10 },	/* 0x04	AK4953A_04_SIGNAL_SELECT3	*/
	{ 0x05, 0x02 },	/* 0x05	AK4953A_05_MODE_CONTROL1	*/
	{ 0x06, 0x00 },	/* 0x06	AK4953A_06_MODE_CONTROL2	*/
	{ 0x07, 0x1D },	/* 0x07	AK4953A_07_MODE_CONTROL3	*/
	{ 0x08, 0x00 },	/* 0x08	AK4953A_08_DIGITL_MIC		*/
	{ 0x09, 0x01 },	/* 0x09	AK4953A_09_TIMER_SELECT		*/
	{ 0x0a, 0x30 },	/* 0x0A	AK4953A_0A_ALC_TIMER_SELECT	*/
	{ 0x0b, 0x81 },	/* 0x0B	AK4953A_0B_ALC_MODE_CONTROL1	*/
	{ 0x0c, 0xE1 },	/* 0x0C	AK4953A_0C_ALC_MODE_CONTROL2	*/
	{ 0x0d, 0x28 },	/* 0x0D	AK4953A_0D_ALC_MODE_CONTROL3	*/
	{ 0x0e, 0x91 },	/* 0x0E	AK4953A_0E_ALC_VOLUME		*/
	{ 0x0f, 0x91 },	/* 0x0F	AK4953A_0F_LCH_INPUT_VOLUME_CONTROL	*/
	{ 0x10, 0x91 },	/* 0x10	AK4953A_10_RCH_INPUT_VOLUME_CONTROL	*/
	{ 0x11, 0x91 },	/* 0x11	AK4953A_11_LCH_OUTPUT_VOLUME_CONTROL	*/
	{ 0x12, 0x91 },	/* 0x12	AK4953A_12_RCH_OUTPUT_VOLUME_CONTROL	*/
	{ 0x13, 0x18 },	/* 0x13	AK4953A_13_LCH_DIGITAL_VOLUME_CONTROL	*/
	{ 0x14, 0x18 },	/* 0x14	AK4953A_14_RCH_DIGITAL_VOLUME_CONTROL	*/
	{ 0x15, 0x00 },	/* 0x15	AK4953A_15_BEEP_FREQUENCY	*/
	{ 0x16, 0x00 },	/* 0x16	AK4953A_16_BEEP_ON_TIMEL	*/
	{ 0x17, 0x00 },	/* 0x17	AK4953A_17_BEEP_OFF_TIME	*/
	{ 0x18, 0x00 },	/* 0x18	AK4953A_18_BEEP_REPEAT_COUNT	*/
	{ 0x19, 0x00 },	/* 0x19	AK4953A_19_BEEP_VOLUME_CONTROL	*/
	{ 0x1a, 0x00 },	/* 0x1A	AK4953A_1A_RESERVED		*/
	{ 0x1b, 0x00 },	/* 0x1B	AK4953A_1B_RESERVED		*/
	{ 0x1c, 0x01 },	/* 0x1C	AK4953A_1C_DIGITAL_FILTER_SELECT1	*/
	{ 0x1d, 0x03 },	/* 0x1D	AK4953A_1D_DIGITAL_FILTER_MODE	*/
	{ 0x1e, 0xA9 },	/* 0x1E	AK4953A_1E_HPF2_COEFFICIENT0	*/
	{ 0x1f, 0x1F },	/* 0x1F	AK4953A_1F_HPF2_COEFFICIENT1	*/
	{ 0x20, 0xAD },	/* 0x20	AK4953A_20_HPF2_COEFFICIENT2	*/
	{ 0x21, 0x20 },	/* 0x21	AK4953A_21_HPF2_COEFFICIENT3	*/
	{ 0x22, 0x7F },	/* 0x22	AK4953A_22_LPF_COEFFICIENT0	*/
	{ 0x23, 0x0C },	/* 0x23	AK4953A_23_LPF_COEFFICIENT1	*/
	{ 0x24, 0xFF },	/* 0x24	AK4953A_24_LPF_COEFFICIENT2	*/
	{ 0x25, 0x38 },	/* 0x25	AK4953A_25_LPF_COEFFICIENT3	*/
	{ 0x26, 0x00 },	/* 0x26	AK4953A_26_RESERVED		*/
	{ 0x27, 0x00 },	/* 0x27	AK4953A_27_RESERVED		*/
	{ 0x28, 0x00 },	/* 0x28	AK4953A_28_RESERVED		*/
	{ 0x29, 0x00 },	/* 0x29	AK4953A_29_RESERVED		*/
	{ 0x2a, 0x00 },	/* 0x2A	AK4953A_2A_RESERVED		*/
	{ 0x2b, 0x00 },	/* 0x2B	AK4953A_2B_RESERVED		*/
	{ 0x2c, 0x00 },	/* 0x2C	AK4953A_2C_RESERVED		*/
	{ 0x2d, 0x00 },	/* 0x2D	AK4953A_2D_RESERVED		*/
	{ 0x2e, 0x00 },	/* 0x2E	AK4953A_2E_RESERVED		*/
	{ 0x2f, 0x00 },	/* 0x2F	AK4953A_2F_RESERVED		*/
	{ 0x30, 0x00 },	/* 0x30	AK4953A_30_DIGITAL_FILTER_SELECT2	*/
	{ 0x31, 0x00 },	/* 0x31	AK4953A_31_RESERVED		*/
	{ 0x32, 0x68 },	/* 0x32	AK4953A_32_E1_COEFFICIENT0	*/
	{ 0x33, 0x00 },	/* 0x33	AK4953A_33_E1_COEFFICIENT1	*/
	{ 0x34, 0x4F },	/* 0x34	AK4953A_34_E1_COEFFICIENT2	*/
	{ 0x35, 0x3F },	/* 0x35	AK4953A_35_E1_COEFFICIENT3	*/
	{ 0x36, 0xAD },	/* 0x36	AK4953A_36_E1_COEFFICIENT4	*/
	{ 0x37, 0xE0 },	/* 0x37	AK4953A_37_E1_COEFFICIENT5	*/
	{ 0x38, 0x73 },	/* 0x38	AK4953A_38_E2_COEFFICIENT0	*/
	{ 0x39, 0x00 },	/* 0x39	AK4953A_39_E2_COEFFICIENT1	*/
	{ 0x3a, 0x0B },	/* 0x3A	AK4953A_3A_E2_COEFFICIENT2	*/
	{ 0x3b, 0x3F },	/* 0x3B	AK4953A_3B_E2_COEFFICIENT3	*/
	{ 0x3c, 0xE6 },	/* 0x3C	AK4953A_3C_E2_COEFFICIENT4	*/
	{ 0x3d, 0xE0 },	/* 0x3D	AK4953A_3D_E2_COEFFICIENT5	*/
	{ 0x3e, 0x07 },	/* 0x3E	AK4953A_3E_E3_COEFFICIENT0	*/
	{ 0x3f, 0x08 },	/* 0x3F	AK4953A_3F_E3_COEFFICIENT1	*/
	{ 0x40, 0x67 },	/* 0x40	AK4953A_40_E3_COEFFICIENT2	*/
	{ 0x41, 0x37 },	/* 0x41	AK4953A_41_E3_COEFFICIENT3	*/
	{ 0x42, 0x07 },	/* 0x42	AK4953A_42_E3_COEFFICIENT4	*/
	{ 0x43, 0xE8 },	/* 0x43	AK4953A_43_E3_COEFFICIENT5	*/
	{ 0x44, 0xE0 },	/* 0x44	AK4953A_44_E4_COEFFICIENT0	*/
	{ 0x45, 0x0A },	/* 0x45	AK4953A_45_E4_COEFFICIENT1	*/
	{ 0x46, 0xAD },	/* 0x46	AK4953A_46_E4_COEFFICIENT2	*/
	{ 0x47, 0x29 },	/* 0x47	AK4953A_47_E4_COEFFICIENT3	*/
	{ 0x48, 0x80 },	/* 0x48	AK4953A_48_E4_COEFFICIENT4	*/
	{ 0x49, 0xEE },	/* 0x49	AK4953A_49_E4_COEFFICIENT5	*/
	{ 0x4a, 0x04 },	/* 0x4A	AK4953A_4A_E5_COEFFICIENT0	*/
	{ 0x4b, 0x0C },	/* 0x4B	AK4953A_4B_E5_COEFFICIENT1	*/
	{ 0x4c, 0x2B },	/* 0x4C	AK4953A_4C_E5_COEFFICIENT2	*/
	{ 0x4d, 0x15 },	/* 0x4D	AK4953A_4D_E5_COEFFICIENT3	*/
	{ 0x4e, 0x07 },	/* 0x4E	AK4953A_4E_E5_COEFFICIENT4	*/
	{ 0x4f, 0xF4 },	/* 0x4F	AK4953A_4F_E5_COEFFICIENT5	*/
};

/*
 * MIC Gain control:
 * from 11 to 29 dB (dB scale specified with min/max values instead of step)
 */
static DECLARE_TLV_DB_MINMAX(mgain_tlv, 1100, 2900);

/*
 * Input Digital volume L control:
 * from -54.375 to 36 dB in 0.375 dB steps mute instead of -54.375 dB)
 */
static DECLARE_TLV_DB_SCALE(ivoll_tlv, -5437, 37, 1);

/*
 * Input Digital volume R control:
 * from -54.375 to 36 dB in 0.375 dB steps mute instead of -54.375 dB)
 */
static DECLARE_TLV_DB_SCALE(ivolr_tlv, -5437, 37, 1);

/*
 * Output Digital volume L control (Manual mode):
 * (This can be used as Bluetooth I/F output volume)
 * from -57.5 to 6 dB in 0.5 dB steps (mute instead of -57.5 dB)
 */
static DECLARE_TLV_DB_SCALE(dvoll1_tlv, -5437, 37, 1);

/*
 * Output Digital volume R control (Manual mode):
 * (This can be used as Bluetooth I/F output volume)
 * from -57.5 to 6 dB in 0.5 dB steps (mute instead of -57.5 dB)
 */
static DECLARE_TLV_DB_SCALE(dvolr1_tlv, -5437, 37, 1);

/*
 * Output Digital volume L control:
 * (This can be used as Bluetooth I/F output volume)
 * from -57.5 to 6 dB in 0.5 dB steps (mute instead of -57.5 dB)
 */
static DECLARE_TLV_DB_SCALE(dvoll2_tlv, -11550, 50, 1);

/*
 * Output Digital volume R control:
 * (This can be used as Bluetooth I/F output volume)
 * from -57.5 to 6 dB in 0.5 dB steps (mute instead of -57.5 dB)
 */
static DECLARE_TLV_DB_SCALE(dvolr2_tlv, -11550, 50, 1);

/*
 * Speaker output volume control:
 * from -5.3 to 11.3 dB in 2 dB steps (mute instead of -33 dB)
 */
static DECLARE_TLV_DB_SCALE(spkout_tlv, 530, 200, 0);

static const char * const ak4953a_dem_select_texts[] = {
	"44.1kHz", "Off", "48kHz", "32kHz"
};

static const struct soc_enum ak4953a_enum[] = {
	SOC_ENUM_SINGLE(AK4953A_07_MODE_CONTROL3, 0,
			ARRAY_SIZE(ak4953a_dem_select_texts),
			ak4953a_dem_select_texts),
};

static const struct snd_kcontrol_new ak4953a_snd_controls[] = {
	SOC_SINGLE_TLV("Mic Gain Control",
		AK4953A_02_SIGNAL_SELECT1, 0, 0x06, 0, mgain_tlv),
	SOC_SINGLE_TLV("Input Digital Volume L",
		AK4953A_0F_LCH_INPUT_VOLUME_CONTROL, 0, 0xF1, 0, ivoll_tlv),
	SOC_SINGLE_TLV("Input Digital Volume R",
		AK4953A_10_RCH_INPUT_VOLUME_CONTROL, 0, 0xF1, 0, ivolr_tlv),
	SOC_SINGLE_TLV("Digital Output Volume1 L (Manual Mode)",
		AK4953A_11_LCH_OUTPUT_VOLUME_CONTROL, 0, 0xF1, 0, dvoll1_tlv),
	SOC_SINGLE_TLV("Digital Output Volume1 R (Manual Mode)",
		AK4953A_12_RCH_OUTPUT_VOLUME_CONTROL, 0, 0xF1, 0, dvolr1_tlv),
	SOC_SINGLE_TLV("Digital Output Volume2 L",
		AK4953A_13_LCH_DIGITAL_VOLUME_CONTROL, 0, 0xFF, 1, dvoll2_tlv),
	SOC_SINGLE_TLV("Digital Output Volume2 R",
		AK4953A_14_RCH_DIGITAL_VOLUME_CONTROL, 0, 0xFF, 1, dvolr2_tlv),
	SOC_SINGLE_TLV("Speaker Output Volume",
		AK4953A_03_SIGNAL_SELECT2, 6, 0x03, 0, spkout_tlv),

	SOC_SINGLE("Input Volume Dependent", AK4953A_07_MODE_CONTROL3,
		2, 1, 0),
	SOC_SINGLE("Digital Output Volume1 Dependent", AK4953A_07_MODE_CONTROL3,
		3, 1, 0),
	SOC_SINGLE("Digital Output Volume2 Dependent", AK4953A_07_MODE_CONTROL3,
		4, 1, 0),
	SOC_SINGLE("Headphone Monaural Output", AK4953A_04_SIGNAL_SELECT3,
		2, 1, 0),
	SOC_SINGLE("High Path Filter 1", AK4953A_1C_DIGITAL_FILTER_SELECT1,
		1, 3, 0),
	SOC_SINGLE("High Path Filter 2", AK4953A_1C_DIGITAL_FILTER_SELECT1,
		4, 1, 0),
	SOC_SINGLE("Low Path Filter",    AK4953A_1C_DIGITAL_FILTER_SELECT1,
		5, 1, 0),
	SOC_SINGLE("5 Band Equalizer 1", AK4953A_30_DIGITAL_FILTER_SELECT2,
		0, 1, 0),
	SOC_SINGLE("5 Band Equalizer 2", AK4953A_30_DIGITAL_FILTER_SELECT2,
		1, 1, 0),
	SOC_SINGLE("5 Band Equalizer 3", AK4953A_30_DIGITAL_FILTER_SELECT2,
		2, 1, 0),
	SOC_SINGLE("5 Band Equalizer 4", AK4953A_30_DIGITAL_FILTER_SELECT2,
		3, 1, 0),
	SOC_SINGLE("5 Band Equalizer 5", AK4953A_30_DIGITAL_FILTER_SELECT2,
		4, 1, 0),
	SOC_SINGLE("Auto Level Control 1", AK4953A_0B_ALC_MODE_CONTROL1,
		5, 1, 0),
	SOC_SINGLE("Auto Level Control 2", AK4953A_0B_ALC_MODE_CONTROL1,
		6, 1, 0),
	SOC_ENUM("De-emphasis Control", ak4953a_enum[0]),
	SOC_SINGLE("Soft Mute Control",  AK4953A_07_MODE_CONTROL3,
		5, 1, 0),
};

static const char * const ak4953a_adc1_select_texts[] = {
	"Stereo", "Mono"
};

static const struct soc_enum ak4953a_adc1_mux_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(ak4953a_adc1_select_texts),
			ak4953a_adc1_select_texts);

static const struct snd_kcontrol_new ak4953a_adc1_mux_control =
	SOC_DAPM_ENUM("ADC Switch1", ak4953a_adc1_mux_enum);

static const char * const ak4953a_adc2_select_texts[] = {
	"Stereo", "Mono"
};

static const struct soc_enum ak4953a_adc2_mux_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(ak4953a_adc2_select_texts),
			ak4953a_adc2_select_texts);

static const struct snd_kcontrol_new ak4953a_adc2_mux_control =
	SOC_DAPM_ENUM("ADC Switch2", ak4953a_adc2_mux_enum);

static const char * const ak4953a_line_texts[] = {
	"LRIN1", "LRIN2", "LRIN3"
};

static const unsigned int ak4953a_select_values[] = {
	0x00, 0x03, 0x0c
};

static const struct soc_enum ak4953a_input_mux_enum =
	SOC_VALUE_ENUM_SINGLE(AK4953A_03_SIGNAL_SELECT2, 0, 0x0f,
			ARRAY_SIZE(ak4953a_line_texts),
			ak4953a_line_texts,
			ak4953a_select_values);

static const struct snd_kcontrol_new ak4953a_input_select_controls =
	SOC_DAPM_ENUM("Input Select", ak4953a_input_mux_enum);

static const char * const ak4953a_in1_select_texts[] = {
	"IN1", "Mic Bias"
};

static const struct soc_enum ak4953a_in1_mux_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(ak4953a_in1_select_texts),
			ak4953a_in1_select_texts);

static const struct snd_kcontrol_new ak4953a_in1_mux_control =
	SOC_DAPM_ENUM("IN1 Switch", ak4953a_in1_mux_enum);

static const char * const ak4953a_in2_select_texts[] = {
	"IN2", "Mic Bias"
};

static const struct soc_enum ak4953a_in2_mux_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(ak4953a_in2_select_texts),
			ak4953a_in2_select_texts);

static const struct snd_kcontrol_new ak4953a_in2_mux_control =
	SOC_DAPM_ENUM("IN2 Switch", ak4953a_in2_mux_enum);

static const char * const ak4953a_micbias_select_texts[] = {
	"IN1", "IN2"
};

static const struct soc_enum ak4953a_micbias_mux_enum =
	SOC_ENUM_SINGLE(AK4953A_02_SIGNAL_SELECT1, 4,
			ARRAY_SIZE(ak4953a_micbias_select_texts),
			ak4953a_micbias_select_texts);

static const struct snd_kcontrol_new ak4953a_micbias_mux_control =
	SOC_DAPM_ENUM("MIC bias Select", ak4953a_micbias_mux_enum);

static const char * const ak4953a_adcpf_select_texts[] = {
	"SDTI", "ADC"
};

static const struct soc_enum ak4953a_adcpf_mux_enum =
	SOC_ENUM_SINGLE(AK4953A_1D_DIGITAL_FILTER_MODE, 1,
			ARRAY_SIZE(ak4953a_adcpf_select_texts),
			ak4953a_adcpf_select_texts);

static const struct snd_kcontrol_new ak4953a_adcpf_mux_control =
	SOC_DAPM_ENUM("ADCPF Select", ak4953a_adcpf_mux_enum);

static const char * const ak4953a_pfsdo_select_texts[] = {
	"ADC", "PFIL"
};

static const struct soc_enum ak4953a_pfsdo_mux_enum =
	SOC_ENUM_SINGLE(AK4953A_1D_DIGITAL_FILTER_MODE, 0,
			ARRAY_SIZE(ak4953a_pfsdo_select_texts),
			ak4953a_pfsdo_select_texts);

static const struct snd_kcontrol_new ak4953a_pfsdo_mux_control =
	SOC_DAPM_ENUM("PFSDO Select", ak4953a_pfsdo_mux_enum);

static const char * const ak4953a_pfdac_select_texts[] = {
	"SDTI", "PFIL"
};

static const struct soc_enum ak4953a_pfdac_mux_enum =
	SOC_ENUM_SINGLE(AK4953A_1D_DIGITAL_FILTER_MODE, 2,
			ARRAY_SIZE(ak4953a_pfdac_select_texts),
			ak4953a_pfdac_select_texts);

static const struct snd_kcontrol_new ak4953a_pfdac_mux_control =
	SOC_DAPM_ENUM("PFDAC Select", ak4953a_pfdac_mux_enum);

static const char * const ak4953a_mic_select_texts[] = {
	"AMIC", "DMIC"
};

static const struct soc_enum ak4953a_mic_mux_enum =
	SOC_ENUM_SINGLE(AK4953A_08_DIGITL_MIC, 0,
			ARRAY_SIZE(ak4953a_mic_select_texts),
			ak4953a_mic_select_texts);

static const struct snd_kcontrol_new ak4953a_mic_mux_control =
	SOC_DAPM_ENUM("MIC Select", ak4953a_mic_mux_enum);

static const char * const ak4953a_hpsw_select_texts[] = {
	"Off", "On"
};

static const struct soc_enum ak4953a_hpsw_mux_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(ak4953a_hpsw_select_texts),
			ak4953a_hpsw_select_texts);

static const struct snd_kcontrol_new ak4953a_hpsw_mux_control =
	SOC_DAPM_ENUM("HP Switch", ak4953a_hpsw_mux_enum);

static const struct snd_kcontrol_new ak4953a_dacs_mixer_controls[] = {
	SOC_DAPM_SINGLE("DACS", AK4953A_02_SIGNAL_SELECT1, 5, 1, 0),
};

static int ak4953a_spko_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
		mdelay(1);
		snd_soc_update_bits(codec, AK4953A_02_SIGNAL_SELECT1,
				0x80, 0x80);
		break;
	case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
		snd_soc_update_bits(codec, AK4953A_02_SIGNAL_SELECT1,
				0x80, 0x00);
		mdelay(1);
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget ak4953a_dapm_widgets[] = {

#ifdef PLL_32BICK_MODE
	SND_SOC_DAPM_SUPPLY("PMPLL", AK4953A_01_POWER_MANAGEMENT2, 0, 0,
			NULL, 0),
#else
#ifdef PLL_64BICK_MODE
	SND_SOC_DAPM_SUPPLY("PMPLL", AK4953A_01_POWER_MANAGEMENT2, 0, 0,
			NULL, 0),
#endif
#endif

	/* DAC */
	SND_SOC_DAPM_DAC("DAC", "NULL", AK4953A_00_POWER_MANAGEMENT1, 2, 0),

	/* Analog Output */
	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_PGA("SPK Amp", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPL Amp", AK4953A_01_POWER_MANAGEMENT2, 5, 0,
			NULL, 0),
	SND_SOC_DAPM_PGA("HPR Amp", AK4953A_01_POWER_MANAGEMENT2, 4, 0,
			NULL, 0),
	SND_SOC_DAPM_MIXER_E("SPKO Mixer", AK4953A_00_POWER_MANAGEMENT1, 4, 0,
			&ak4953a_dacs_mixer_controls[0],
			ARRAY_SIZE(ak4953a_dacs_mixer_controls),
			ak4953a_spko_event, (SND_SOC_DAPM_POST_PMU |
					SND_SOC_DAPM_PRE_PMD |
					SND_SOC_DAPM_PRE_PMU |
					SND_SOC_DAPM_POST_PMD)),
	SND_SOC_DAPM_MUX("DACHP", SND_SOC_NOPM, 0, 0,
			&ak4953a_hpsw_mux_control),
	SND_SOC_DAPM_AIF_OUT("SDTO", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),

	/* PFIL */
	SND_SOC_DAPM_MUX("PFDAC MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_pfdac_mux_control),
	SND_SOC_DAPM_MUX("PFSDO MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_pfsdo_mux_control),
	SND_SOC_DAPM_MUX("ADCPF MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_adcpf_mux_control),

	/* Digital Mic */
	SND_SOC_DAPM_INPUT("DMICLIN"),
	SND_SOC_DAPM_INPUT("DMICRIN"),
	SND_SOC_DAPM_ADC("DMICL", "NULL", AK4953A_08_DIGITL_MIC, 4, 0),
	SND_SOC_DAPM_ADC("DMICR", "NULL", AK4953A_08_DIGITL_MIC, 5, 0),
	SND_SOC_DAPM_MUX("MIC MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_mic_mux_control),
	SND_SOC_DAPM_MUX("ADC MUX2", SND_SOC_NOPM, 0, 0,
			&ak4953a_adc2_mux_control),

	/* ADC */
	SND_SOC_DAPM_ADC("ADC Left", "NULL", AK4953A_00_POWER_MANAGEMENT1,
			0, 0),
	SND_SOC_DAPM_ADC("ADC Right", "NULL", AK4953A_00_POWER_MANAGEMENT1,
			1, 0),
	SND_SOC_DAPM_MUX("ADC MUX1", SND_SOC_NOPM, 0, 0,
			&ak4953a_adc1_mux_control),
	SND_SOC_DAPM_ADC("PFIL", "NULL", AK4953A_00_POWER_MANAGEMENT1, 7, 0),

	/* Analog Input  MIC Bias */
	SND_SOC_DAPM_INPUT("LRIN1"),
	SND_SOC_DAPM_INPUT("LRIN2"),
	SND_SOC_DAPM_INPUT("LRIN3"),
	SND_SOC_DAPM_MICBIAS("Mic Bias", AK4953A_02_SIGNAL_SELECT1, 3, 0),
	SND_SOC_DAPM_MUX("IN2 MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_in2_mux_control),
	SND_SOC_DAPM_MUX("IN1 MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_in1_mux_control),
	SND_SOC_DAPM_MUX("Input Select MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_input_select_controls),
	SND_SOC_DAPM_MUX("Mic Bias MUX", SND_SOC_NOPM, 0, 0,
			&ak4953a_micbias_mux_control),
};

static const struct snd_soc_dapm_route ak4953a_intercon[] = {

#ifdef PLL_32BICK_MODE
	{"ADC Left", "NULL", "PMPLL"},
	{"ADC Right", "NULL", "PMPLL"},
	{"DAC", "NULL", "PMPLL"},
#else
#ifdef PLL_64BICK_MODE
	{"ADC Left", "NULL", "PMPLL"},
	{"ADC Right", "NULL", "PMPLL"},
	{"DAC", "NULL", "PMPLL"},
#endif
#endif
	{"Mic Bias MUX", "IN1", "LRIN1"},
	{"Mic Bias MUX", "IN2", "LRIN2"},

	{"Mic Bias", "NULL", "Mic Bias MUX"},

	{"IN1 MUX", "IN1", "LRIN1"},
	{"IN1 MUX", "Mic Bias", "Mic Bias"},
	{"IN2 MUX", "IN2", "LRIN2"},
	{"IN2 MUX", "Mic Bias", "Mic Bias"},

	{"Input Select MUX", "LRIN1", "IN1 MUX"},
	{"Input Select MUX", "LRIN2", "IN2 MUX"},
	{"Input Select MUX", "LRIN3", "LRIN3"},

	{"ADC Left", "NULL", "Input Select MUX"},
	{"ADC Right", "NULL", "Input Select MUX"},

	{"ADC MUX1", "Stereo", "ADC Left"},
	{"ADC MUX1", "Stereo", "ADC Right"},
	{"ADC MUX1", "Mono", "ADC Left"},

	{"DMICL", "NULL", "DMICLIN"},
	{"DMICR", "NULL", "DMICRIN"},

	{"ADC MUX2", "Stereo", "DMICL"},
	{"ADC MUX2", "Stereo", "DMICR"},
	{"ADC MUX2", "Mono", "DMICL"},

	{"MIC MUX", "DMIC", "ADC MUX2"},
	{"MIC MUX", "AMIC", "ADC MUX1"},

	{"ADCPF MUX", "ADC", "MIC MUX"},
	{"ADCPF MUX", "SDTI", "SDTI"},
	{"PFIL", "NULL", "ADCPF MUX"},

	{"PFSDO MUX", "PFIL", "PFIL"},
	{"PFSDO MUX", "ADC", "MIC MUX"},
	{"SDTO", "NULL", "PFSDO MUX"},

	{"PFDAC MUX", "PFIL", "PFIL"},
	{"PFDAC MUX", "SDTI", "SDTI"},
	{"DAC", "NULL", "PFDAC MUX"},

	{"DACHP", "On", "DAC"},
	{"HPL Amp", "NULL", "DACHP"},
	{"HPR Amp", "NULL", "DACHP"},
	{"HPL", "NULL", "HPL Amp"},
	{"HPR", "NULL", "HPR Amp"},

	{"SPKO Mixer", "DACS", "DAC"},
	{"SPK Amp", "NULL", "SPKO Mixer"},
	{"SPK", "NULL", "SPK Amp"},
};

static int ak4953a_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 fs;

	fs = snd_soc_read(codec, AK4953A_06_MODE_CONTROL2);
	fs &= ~AK4953A_FS;

	switch (params_rate(params)) {
	case 8000:
	case 11025:
	case 12000:
		fs |= AK4953A_FS_12KHZ;
		break;
	case 16000:
	case 22050:
	case 24000:
		fs |= AK4953A_FS_24KHZ;
		break;
	case 32000:
	case 44100:
	case 48000:
		fs |= AK4953A_FS_48KHZ;
		break;
	case 96000:
		fs |= AK4953A_FS_96KHZ;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, AK4953A_06_MODE_CONTROL2, fs);

	return 0;
}

static int ak4953a_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 pll;

	pll = snd_soc_read(codec, AK4953A_05_MODE_CONTROL1);
	pll &= ~AK4953A_PLL;

#ifdef PLL_32BICK_MODE
	pll |= AK4953A_PLL_BICK32;
#else
#ifdef PLL_64BICK_MODE
	pll |= AK493A_PLL_BICK64;
#else
	pll |= AK4953A_EXT_SLAVE;
#endif
#endif

	snd_soc_write(codec, AK4953A_05_MODE_CONTROL1, pll);

	return 0;
}

static int ak4953a_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 mode;
	u8 format;

	/* set master/slave audio interface */
	mode = snd_soc_read(codec, AK4953A_01_POWER_MANAGEMENT2);
#ifdef PLL_32BICK_MODE
	mode |= AK4953A_PMPLL;
#else
#ifdef PLL_64BICK_MODE
	mode |= AK4953A_PMPLL;
#endif
#endif
	format = snd_soc_read(codec, AK4953A_05_MODE_CONTROL1);
	format &= ~AK4953A_DIF;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		mode &= ~(AK4953A_M_S);
		format &= ~(AK4953A_BCKO);
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		mode |= (AK4953A_M_S);
		format |= (AK4953A_BCKO);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
	default:
		dev_err(codec->dev, "Clock mode unsupported");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= AK4953A_DIF_I2S_MODE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= AK4953A_DIF_24MSB_MODE;
		break;
	default:
		return -EINVAL;
	}

	/* set mode and format */

	snd_soc_write(codec, AK4953A_01_POWER_MANAGEMENT2, mode);
	snd_soc_write(codec, AK4953A_05_MODE_CONTROL1, format);

	return 0;
}

static int ak4953a_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		ret = snd_soc_update_bits(codec, AK4953A_00_POWER_MANAGEMENT1,
				AK4953A_PMVCM, AK4953A_PMVCM);
		break;
	case SND_SOC_BIAS_OFF:
		ret = snd_soc_write(codec, AK4953A_00_POWER_MANAGEMENT1, 0x00);
		break;
	}
	codec->dapm.bias_level = level;

	if (ret < 0)
		return ret;

	return 0;
}

#define AK4953A_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
				SNDRV_PCM_RATE_96000)

#define AK4953A_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops ak4953a_dai_ops = {
	.hw_params	= ak4953a_hw_params,
	.set_sysclk	= ak4953a_set_dai_sysclk,
	.set_fmt	= ak4953a_set_dai_fmt,
};

struct snd_soc_dai_driver ak4953a_dai = {
	.name = "ak4953a-AIF1",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AK4953A_RATES,
		.formats = AK4953A_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AK4953A_RATES,
		.formats = AK4953A_FORMATS,
	},
	.ops = &ak4953a_dai_ops,
};

static int ak4953a_set_reg_digital_effect(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, AK4953A_09_TIMER_SELECT,
			AK4953A_14_RCH_DIGITAL_VOLUME_CONTROL);
	snd_soc_write(codec, AK4953A_1C_DIGITAL_FILTER_SELECT1,
			AK4953A_1C_DIGITAL_FILTER_SELECT1);
	snd_soc_write(codec, AK4953A_1E_HPF2_COEFFICIENT0,
			AK4953A_25_LPF_COEFFICIENT3);
	snd_soc_write(codec, AK4953A_32_E1_COEFFICIENT0,
			AK4953A_4F_E5_COEFFICIENT5);

	return 0;
}

static int ak4953a_probe(struct snd_soc_codec *codec)
{
	int ret;

	snd_soc_write(codec, AK4953A_00_POWER_MANAGEMENT1, 0x00);
	snd_soc_write(codec, AK4953A_00_POWER_MANAGEMENT1, 0x00);

	ret = ak4953a_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (ret)
		return ret;

	ak4953a_set_reg_digital_effect(codec);

	return 0;
}

static int ak4953a_remove(struct snd_soc_codec *codec)
{
	ak4953a_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_ak4953a = {
	.probe = ak4953a_probe,
	.remove = ak4953a_remove,
	.set_bias_level = ak4953a_set_bias_level,
	.controls = ak4953a_snd_controls,
	.num_controls = ARRAY_SIZE(ak4953a_snd_controls),
	.dapm_widgets = ak4953a_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4953a_dapm_widgets),
	.dapm_routes = ak4953a_intercon,
	.num_dapm_routes = ARRAY_SIZE(ak4953a_intercon),
};

static const struct regmap_config ak4953a_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK4953A_4F_E5_COEFFICIENT5,
	.reg_defaults = ak4953a_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4953a_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

static int ak4953a_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client, &ak4953a_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&client->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	if (np) {
		int gpio = of_get_named_gpio(np, "reset-gpio", 0);

		if (gpio_is_valid(gpio)) {
			ret = devm_gpio_request_one(&client->dev, gpio,
				     GPIOF_OUT_INIT_LOW,
				     "ak4953a reset");
			if (ret < 0)
				return ret;

			udelay(2);
			gpio_set_value(gpio, 1);
		}
	}

	ret = snd_soc_register_codec(&client->dev,
			&soc_codec_dev_ak4953a, &ak4953a_dai, 1);
	return ret;
}

static int ak4953a_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct of_device_id ak4953a_of_match[] = {
	{ .compatible = "asahi-kasei,ak4953a", },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4953a_of_match);

static const struct i2c_device_id ak4953a_id_table[] = {
	{ "ak4953a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4953a_id_table);

static struct i2c_driver ak4953a_i2c_driver = {
	.driver = {
		.name = "ak4953a",
		.owner = THIS_MODULE,
		.of_match_table = ak4953a_of_match,
	},
	.probe = ak4953a_i2c_probe,
	.remove = ak4953a_i2c_remove,
	.id_table = ak4953a_id_table,
};

module_i2c_driver(ak4953a_i2c_driver);

MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_DESCRIPTION("Asahi Kasei AK4953A ALSA SoC driver");
MODULE_LICENSE("GPL");
