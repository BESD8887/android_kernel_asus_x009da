/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/spmi.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/platform_device.h>

/* Min ADC code represets 0V */
#define QPNP_VADC_MIN_ADC_CODE			0x6000
/* Max ADC code represents full-scale range of 1.8V */
#define QPNP_VADC_MAX_ADC_CODE			0xA800
#define KELVINMIL_DEGMIL	273160

/* Units for temperature below (on x axis) is in 0.1DegC as
   required by the battery driver. Note the resolution used
   here to compute the table was done for DegC to milli-volts.
   In consideration to limit the size of the table for the given
   temperature range below, the result is linearly interpolated
   and provided to the battery driver in the units desired for
   their framework which is 0.1DegC. True resolution of 0.1DegC
   will result in the below table size to increase by 10 times */
/*[Arima_5830][bozhi_lin] update Battery Thermal_NTC ADC table 20160407 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_ER1 && defined(CONFIG_BSP_HW_SKU_5830))
/*[Arima_5830][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160525 begin*/
static const struct qpnp_vadc_map_pt adcmap_btm_threshold_pr[] = {
	{-400,	1653},
	{-390,	1646},
	{-380,	1638},
	{-370,	1630},
	{-360,	1622},
	{-350,	1614},
	{-340,	1605},
	{-330,	1596},
	{-320,	1587},
	{-310,	1577},
	{-300,	1567},
	{-290,	1557},
	{-280,	1547},
	{-270,	1536},
	{-260,	1526},
	{-250,	1515},
	{-240,	1503},
	{-230,	1492},
	{-220,	1480},
	{-210,	1468},
	{-200,	1456},
	{-190,	1444},
	{-180,	1432},
	{-170,	1419},
	{-160,	1407},
	{-150,	1394},
	{-140,	1381},
	{-130,	1368},
	{-120,	1355},
	{-110,	1341},
	{-100,	1328},
	{ -90,	1314},
	{ -80,	1301},
	{ -70,	1287},
	{ -60,	1274},
	{ -50,	1260},
	{ -40,	1247},
	{ -30,	1233},
	{ -20,	1220},
	{ -10,	1206},
	{   0,	1193},
	{  10,	1180},
	{  20,	1167},
	{  30,	1153},
	{  40,	1140},
	{  50,	1128},
	{  60,	1115},
	{  70,	1102},
	{  80,	1090},
	{  90,	1077},
	{ 100,	1065},
	{ 110,	1053},
	{ 120,	1041},
	{ 130,	1030},
	{ 140,	1018},
	{ 150,	1007},
	{ 160,	996},
	{ 170,	985},
	{ 180,	974},
	{ 190,	963},
	{ 200,	953},
	{ 210,	943},
	{ 220,	933},
	{ 230,	923},
	{ 240,	914},
	{ 250,	904},
	{ 260,	895},
	{ 270,	886},
	{ 280,	878},
	{ 290,	869},
	{ 300,	861},
	{ 310,	853},
	{ 320,	845},
	{ 330,	837},
	{ 340,	829},
	{ 350,	822},
	{ 360,	815},
	{ 370,	808},
	{ 380,	801},
	{ 390,	795},
	{ 400,	788},
	{ 410,	782},
	{ 420,	776},
	{ 430,	770},
	{ 440,	764},
	{ 450,	758},
	{ 460,	753},
	{ 470,	747},
	{ 480,	742},
	{ 490,	737},
	{ 500,	732},
	{ 510,	728},
	{ 520,	723},
	{ 530,	718},
	{ 540,	714},
	{ 550,	710},
	{ 560,	706},
	{ 570,	702},
	{ 580,	698},
	{ 590,	694},
	{ 600,	690},
	{ 610,	687},
	{ 620,	684},
	{ 630,	680},
	{ 640,	677},
	{ 650,	674},
	{ 660,	671},
	{ 670,	668},
	{ 680,	665},
	{ 690,	662},
	{ 700,	660},
	{ 710,	657},
	{ 720,	655},
	{ 730,	652},
	{ 740,	650},
	{ 750,	647},
	{ 760,	645},
	{ 770,	643},
	{ 780,	641},
	{ 790,	639},
	{ 800,	637},
	{ 810,	635},
	{ 820,	633},
	{ 830,	631},
	{ 840,	630},
	{ 850,	628},
	{ 860,	626},
	{ 870,	625},
	{ 880,	623},
	{ 890,	621},
	{ 900,	620},
	{ 910,	618},
	{ 920,	617},
	{ 930,	616},
	{ 940,	614},
	{ 950,	613},
	{ 960,	612},
	{ 970,	611},
	{ 980,	609},
	{ 990,	608},
	{1000,	607},
	{1010,	606},
	{1020,	605},
	{1030,	604},
	{1040,	603},
	{1050,	602},
	{1060,	601},
	{1070,	600},
	{1080,	599},
	{1090,	598},
	{1100,	598},
	{1110,	597},
	{1120,	596},
	{1130,	595},
	{1140,	594},
	{1150,	594},
	{1160,	593},
	{1170,	592},
	{1180,	592},
	{1190,	591},
	{1200,	590},
	{1210,	590},
	{1220,	589},
	{1230,	589},
	{1240,	588},
	{1250,	587}
};
/*[Arima_5830][bozhi_lin] 20160525 end*/
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-400,	1694},
	{-390,	1689},
	{-380,	1683},
	{-370,	1677},
	{-360,	1670},
	{-350,	1664},
	{-340,	1657},
	{-330,	1650},
	{-320,	1643},
	{-310,	1635},
	{-300,	1627},
	{-290,	1619},
	{-280,	1611},
	{-270,	1602},
	{-260,	1594},
	{-250,	1585},
	{-240,	1575},
	{-230,	1566},
	{-220,	1556},
	{-210,	1546},
	{-200,	1535},
	{-190,	1525},
	{-180,	1514},
	{-170,	1503},
	{-160,	1492},
	{-150,	1480},
	{-140,	1469},
	{-130,	1457},
	{-120,	1445},
	{-110,	1432},
	{-100,	1420},
	{ -90,	1407},
	{ -80,	1394},
	{ -70,	1381},
	{ -60,	1368},
	{ -50,	1355},
	{ -40,	1341},
	{ -30,	1328},
	{ -20,	1314},
	{ -10,	1300},
	{   0,	1287},
	{  10,	1273},
	{  20,	1259},
	{  30,	1245},
	{  40,	1231},
	{  50,	1217},
	{  60,	1203},
	{  70,	1189},
	{  80,	1175},
	{  90,	1161},
	{ 100,	1147},
	{ 110,	1134},
	{ 120,	1120},
	{ 130,	1106},
	{ 140,	1092},
	{ 150,	1079},
	{ 160,	1065},
	{ 170,	1052},
	{ 180,	1039},
	{ 190,	1026},
	{ 200,	1013},
	{ 210,	1000},
	{ 220,	988},
	{ 230,	975},
	{ 240,	963},
	{ 250,	951},
	{ 260,	939},
	{ 270,	927},
	{ 280,	915},
	{ 290,	904},
	{ 300,	893},
	{ 310,	882},
	{ 320,	871},
	{ 330,	860},
	{ 340,	850},
	{ 350,	839},
	{ 360,	829},
	{ 370,	819},
	{ 380,	810},
	{ 390,	800},
	{ 400,	791},
	{ 410,	782},
	{ 420,	773},
	{ 430,	764},
	{ 440,	756},
	{ 450,	748},
	{ 460,	739},
	{ 470,	731},
	{ 480,	724},
	{ 490,	716},
	{ 500,	709},
	{ 510,	701},
	{ 520,	694},
	{ 530,	687},
	{ 540,	681},
	{ 550,	674},
	{ 560,	668},
	{ 570,	662},
	{ 580,	656},
	{ 590,	650},
	{ 600,	644},
	{ 610,	638},
	{ 620,	633},
	{ 630,	628},
	{ 640,	623},
	{ 650,	618},
	{ 660,	613},
	{ 670,	608},
	{ 680,	604},
	{ 690,	599},
	{ 700,	595},
	{ 710,	591},
	{ 720,	586},
	{ 730,	582},
	{ 740,	579},
	{ 750,	575},
	{ 760,	571},
	{ 770,	568},
	{ 780,	564},
	{ 790,	561},
	{ 800,	557},
	{ 810,	554},
	{ 820,	551},
	{ 830,	548},
	{ 840,	545},
	{ 850,	542},
	{ 860,	539},
	{ 870,	537},
	{ 880,	534},
	{ 890,	531},
	{ 900,	529},
	{ 910,	527},
	{ 920,	524},
	{ 930,	522},
	{ 940,	520},
	{ 950,	517},
	{ 960,	515},
	{ 970,	513},
	{ 980,	511},
	{ 990,	509},
	{1000,	507},
	{1010,	506},
	{1020,	504},
	{1030,	502},
	{1040,	500},
	{1050,	499},
	{1060,	497},
	{1070,	495},
	{1080,	494},
	{1090,	492},
	{1100,	491},
	{1110,	490},
	{1120,	488},
	{1130,	487},
	{1140,	486},
	{1150,	484},
	{1160,	483},
	{1170,	482},
	{1180,	481},
	{1190,	480},
	{1200,	479},
	{1210,	477},
	{1220,	476},
	{1230,	475},
	{1240,	474},
	{1250,	473}
};
/*[Arima_5830][bozhi_lin] 20160407 end*/
/*[Arima_5830][bozhi_lin] workaround for always set battery temp to 25 degreeC 20160125 begin*/
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{250,	1642},
	{250,	866},
	{250,	203}
};
/*[Arima_5833][bozhi_lin] apply battery thermal adc table 20160419 begin*/
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
/*[Arima_5833][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160523 begin*/
static const struct qpnp_vadc_map_pt adcmap_btm_threshold_er2[] = {
	{-400,	1674},
	{-390,	1668},
	{-380,	1661},
	{-370,	1654},
	{-360,	1648},
	{-350,	1640},
	{-340,	1633},
	{-330,	1626},
	{-320,	1618},
	{-310,	1610},
	{-300,	1602},
	{-290,	1593},
	{-280,	1585},
	{-270,	1576},
	{-260,	1567},
	{-250,	1558},
	{-240,	1548},
	{-230,	1538},
	{-220,	1528},
	{-210,	1518},
	{-200,	1508},
	{-190,	1497},
	{-180,	1487},
	{-170,	1476},
	{-160,	1465},
	{-150,	1453},
	{-140,	1442},
	{-130,	1430},
	{-120,	1418},
	{-110,	1406},
	{-100,	1393},
	{ -90,	1381},
	{ -80,	1368},
	{ -70,	1356},
	{ -60,	1343},
	{ -50,	1330},
	{ -40,	1316},
	{ -30,	1303},
	{ -20,	1289},
	{ -10,	1276},
	{   0,	1262},
	{  10,	1248},
	{  20,	1235},
	{  30,	1221},
	{  40,	1207},
	{  50,	1193},
	{  60,	1179},
	{  70,	1165},
	{  80,	1152},
	{  90,	1138},
	{ 100,	1124},
	{ 110,	1110},
	{ 120,	1097},
	{ 130,	1083},
	{ 140,	1070},
	{ 150,	1057},
	{ 160,	1044},
	{ 170,	1031},
	{ 180,	1018},
	{ 190,	1005},
	{ 200,	 992},
	{ 210,	 980},
	{ 220,	 968},
	{ 230,	 956},
	{ 240,	 944},
	{ 250,	 932},
	{ 260,	 921},
	{ 270,	 909},
	{ 280,	 898},
	{ 290,	 887},
	{ 300,	 876},
	{ 310,	 866},
	{ 320,	 855},
	{ 330,	 845},
	{ 340,	 835},
	{ 350,	 826},
	{ 360,	 816},
	{ 370,	 807},
	{ 380,	 798},
	{ 390,	 789},
	{ 400,	 780},
	{ 410,	 771},
	{ 420,	 763},
	{ 430,	 755},
	{ 440,	 747},
	{ 450,	 739},
	{ 460,	 732},
	{ 470,	 724},
	{ 480,	 717},
	{ 490,	 710},
	{ 500,	 703},
	{ 510,	 697},
	{ 520,	 690},
	{ 530,	 684},
	{ 540,	 678},
	{ 550,	 672},
	{ 560,	 666},
	{ 570,	 660},
	{ 580,	 655},
	{ 590,	 649},
	{ 600,	 644},
	{ 610,	 639},
	{ 620,	 634},
	{ 630,	 629},
	{ 640,	 624},
	{ 650,	 620},
	{ 660,	 615},
	{ 670,	 611},
	{ 680,	 607},
	{ 690,	 603},
	{ 700,	 599},
	{ 710,	 595},
	{ 720,	 591},
	{ 730,	 587},
	{ 740,	 584},
	{ 750,	 580},
	{ 760,	 577},
	{ 770,	 574},
	{ 780,	 570},
	{ 790,	 567},
	{ 800,	 564},
	{ 810,	 562},
	{ 820,	 559},
	{ 830,	 556},
	{ 840,	 553},
	{ 850,	 551},
	{ 860,	 548},
	{ 870,	 546},
	{ 880,	 543},
	{ 890,	 541},
	{ 900,	 539},
	{ 910,	 537},
	{ 920,	 535},
	{ 930,	 532},
	{ 940,	 530},
	{ 950,	 529},
	{ 960,	 527},
	{ 970,	 525},
	{ 980,	 523},
	{ 990,	 521},
	{1000,	 520},
	{1010,	 518},
	{1020,	 516},
	{1030,	 515},
	{1040,	 513},
	{1050,	 512},
	{1060,	 510},
	{1070,	 509},
	{1080,	 508},
	{1090,	 506},
	{1100,	 505},
	{1110,	 504},
	{1120,	 502},
	{1130,	 501},
	{1140,	 500},
	{1150,	 499},
	{1160,	 498},
	{1170,	 497},
	{1180,	 496},
	{1190,	 495},
	{1200,	 494},
	{1210,	 493},
	{1220,	 492},
	{1230,	 491},
	{1240,	 490},
	{1250,	 489}
};
/*[Arima_5833][bozhi_lin] 20160523 end*/
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-400,	1681},
	{-390,	1675},
	{-380,	1669},
	{-370,	1662},
	{-360,	1656},
	{-350,	1649},
	{-340,	1642},
	{-330,	1634},
	{-320,	1627},
	{-310,	1619},
	{-300,	1611},
	{-290,	1602},
	{-280,	1594},
	{-270,	1585},
	{-260,	1576},
	{-250,	1567},
	{-240,	1557},
	{-230,	1547},
	{-220,	1537},
	{-210,	1527},
	{-200,	1517},
	{-190,	1506},
	{-180,	1495},
	{-170,	1483},
	{-160,	1472},
	{-150,	1460},
	{-140,	1448},
	{-130,	1436},
	{-120,	1423},
	{-110,	1410},
	{-100,	1397},
	{ -90,	1384},
	{ -80,	1370},
	{ -70,	1356},
	{ -60,	1342},
	{ -50,	1328},
	{ -40,	1314},
	{ -30,	1299},
	{ -20,	1284},
	{ -10,	1269},
	{   0,	1254},
	{  10,	1238},
	{  20,	1223},
	{  30,	1207},
	{  40,	1191},
	{  50,	1175},
	{  60,	1159},
	{  70,	1143},
	{  80,	1127},
	{  90,	1111},
	{ 100,	1094},
	{ 110,	1078},
	{ 120,	1062},
	{ 130,	1046},
	{ 140,	1030},
	{ 150,	1014},
	{ 160,	 997},
	{ 170,	 982},
	{ 180,	 966},
	{ 190,	 950},
	{ 200,	 934},
	{ 210,	 919},
	{ 220,	 903},
	{ 230,	 888},
	{ 240,	 873},
	{ 250,	 858},
	{ 260,	 843},
	{ 270,	 828},
	{ 280,	 814},
	{ 290,	 799},
	{ 300,	 785},
	{ 310,	 771},
	{ 320,	 758},
	{ 330,	 744},
	{ 340,	 731},
	{ 350,	 718},
	{ 360,	 705},
	{ 370,	 693},
	{ 380,	 680},
	{ 390,	 668},
	{ 400,	 656},
	{ 410,	 645},
	{ 420,	 633},
	{ 430,	 622},
	{ 440,	 611},
	{ 450,	 600},
	{ 460,	 590},
	{ 470,	 579},
	{ 480,	 569},
	{ 490,	 559},
	{ 500,	 550},
	{ 510,	 540},
	{ 520,	 531},
	{ 530,	 522},
	{ 540,	 513},
	{ 550,	 505},
	{ 560,	 496},
	{ 570,	 488},
	{ 580,	 480},
	{ 590,	 472},
	{ 600,	 465},
	{ 610,	 457},
	{ 620,	 450},
	{ 630,	 443},
	{ 640,	 436},
	{ 650,	 429},
	{ 660,	 423},
	{ 670,	 416},
	{ 680,	 410},
	{ 690,	 404},
	{ 700,	 398},
	{ 710,	 392},
	{ 720,	 387},
	{ 730,	 381},
	{ 740,	 376},
	{ 750,	 371},
	{ 760,	 366},
	{ 770,	 361},
	{ 780,	 356},
	{ 790,	 352},
	{ 800,	 347},
	{ 810,	 343},
	{ 820,	 338},
	{ 830,	 334},
	{ 840,	 330},
	{ 850,	 326},
	{ 860,	 323},
	{ 870,	 319},
	{ 880,	 315},
	{ 890,	 312},
	{ 900,	 308},
	{ 910,	 305},
	{ 920,	 302},
	{ 930,	 299},
	{ 940,	 296},
	{ 950,	 293},
	{ 960,	 290},
	{ 970,	 287},
	{ 980,	 284},
	{ 990,	 281},
	{1000,	 279},
	{1010,	 276},
	{1020,	 274},
	{1030,	 271},
	{1040,	 269},
	{1050,	 267},
	{1060,	 265},
	{1070,	 262},
	{1080,	 260},
	{1090,	 258},
	{1100,	 256},
	{1110,	 254},
	{1120,	 252},
	{1130,	 251},
	{1140,	 249},
	{1150,	 247},
	{1160,	 245},
	{1170,	 244},
	{1180,	 242},
	{1190,	 241},
	{1200,	 239},
	{1210,	 238},
	{1220,	 236},
	{1230,	 235},
	{1240,	 233},
	{1250,	 232}
};
/*[Arima_5833][bozhi_lin] 20160419 end*/
#else
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300,	1642},
	{-200,	1544},
	{-100,	1414},
	{0,	1260},
	{10,	1244},
	{20,	1228},
	{30,	1212},
	{40,	1195},
	{50,	1179},
	{60,	1162},
	{70,	1146},
	{80,	1129},
	{90,	1113},
	{100,	1097},
	{110,	1080},
	{120,	1064},
	{130,	1048},
	{140,	1032},
	{150,	1016},
	{160,	1000},
	{170,	985},
	{180,	969},
	{190,	954},
	{200,	939},
	{210,	924},
	{220,	909},
	{230,	894},
	{240,	880},
	{250,	866},
	{260,	852},
	{270,	838},
	{280,	824},
	{290,	811},
	{300,	798},
	{310,	785},
	{320,	773},
	{330,	760},
	{340,	748},
	{350,	736},
	{360,	725},
	{370,	713},
	{380,	702},
	{390,	691},
	{400,	681},
	{410,	670},
	{420,	660},
	{430,	650},
	{440,	640},
	{450,	631},
	{460,	622},
	{470,	613},
	{480,	604},
	{490,	595},
	{500,	587},
	{510,	579},
	{520,	571},
	{530,	563},
	{540,	556},
	{550,	548},
	{560,	541},
	{570,	534},
	{580,	527},
	{590,	521},
	{600,	514},
	{610,	508},
	{620,	502},
	{630,	496},
	{640,	490},
	{650,	485},
	{660,	281},
	{670,	274},
	{680,	267},
	{690,	260},
	{700,	254},
	{710,	247},
	{720,	241},
	{730,	235},
	{740,	229},
	{750,	224},
	{760,	218},
	{770,	213},
	{780,	208},
	{790,	203}
};
#endif
/*[Arima_5830][bozhi_lin] 20160125 end*/

static const struct qpnp_vadc_map_pt adcmap_qrd_btm_threshold[] = {
	{-200,	1540},
	{-180,	1517},
	{-160,	1492},
	{-140,	1467},
	{-120,	1440},
	{-100,	1412},
	{-80,	1383},
	{-60,	1353},
	{-40,	1323},
	{-20,	1292},
	{0,	1260},
	{20,	1228},
	{40,	1196},
	{60,	1163},
	{80,	1131},
	{100,	1098},
	{120,	1066},
	{140,	1034},
	{160,	1002},
	{180,	971},
	{200,	941},
	{220,	911},
	{240,	882},
	{260,	854},
	{280,	826},
	{300,	800},
	{320,	774},
	{340,	749},
	{360,	726},
	{380,	703},
	{400,	681},
	{420,	660},
	{440,	640},
	{460,	621},
	{480,	602},
	{500,	585},
	{520,	568},
	{540,	552},
	{560,	537},
	{580,	523},
	{600,	510},
	{620,	497},
	{640,	485},
	{660,	473},
	{680,	462},
	{700,	452},
	{720,	442},
	{740,	433},
	{760,	424},
	{780,	416},
	{800,	408},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuaa_btm_threshold[] = {
	{-200,	1476},
	{-180,	1450},
	{-160,	1422},
	{-140,	1394},
	{-120,	1365},
	{-100,	1336},
	{-80,	1306},
	{-60,	1276},
	{-40,	1246},
	{-20,	1216},
	{0,	1185},
	{20,	1155},
	{40,	1126},
	{60,	1096},
	{80,	1068},
	{100,	1040},
	{120,	1012},
	{140,	986},
	{160,	960},
	{180,	935},
	{200,	911},
	{220,	888},
	{240,	866},
	{260,	844},
	{280,	824},
	{300,	805},
	{320,	786},
	{340,	769},
	{360,	752},
	{380,	737},
	{400,	722},
	{420,	707},
	{440,	694},
	{460,	681},
	{480,	669},
	{500,	658},
	{520,	648},
	{540,	637},
	{560,	628},
	{580,	619},
	{600,	611},
	{620,	603},
	{640,	595},
	{660,	588},
	{680,	582},
	{700,	575},
	{720,	569},
	{740,	564},
	{760,	559},
	{780,	554},
	{800,	549},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skug_btm_threshold[] = {
	{-200,	1338},
	{-180,	1307},
	{-160,	1276},
	{-140,	1244},
	{-120,	1213},
	{-100,	1182},
	{-80,	1151},
	{-60,	1121},
	{-40,	1092},
	{-20,	1063},
	{0,	1035},
	{20,	1008},
	{40,	982},
	{60,	957},
	{80,	933},
	{100,	910},
	{120,	889},
	{140,	868},
	{160,	848},
	{180,	830},
	{200,	812},
	{220,	795},
	{240,	780},
	{260,	765},
	{280,	751},
	{300,	738},
	{320,	726},
	{340,	714},
	{360,	704},
	{380,	694},
	{400,	684},
	{420,	675},
	{440,	667},
	{460,	659},
	{480,	652},
	{500,	645},
	{520,	639},
	{540,	633},
	{560,	627},
	{580,	622},
	{600,	617},
	{620,	613},
	{640,	608},
	{660,	604},
	{680,	600},
	{700,	597},
	{720,	593},
	{740,	590},
	{760,	587},
	{780,	585},
	{800,	582},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuh_btm_threshold[] = {
	{-200,	1531},
	{-180,	1508},
	{-160,	1483},
	{-140,	1458},
	{-120,	1432},
	{-100,	1404},
	{-80,	1377},
	{-60,	1348},
	{-40,	1319},
	{-20,	1290},
	{0,	1260},
	{20,	1230},
	{40,	1200},
	{60,	1171},
	{80,	1141},
	{100,	1112},
	{120,	1083},
	{140,	1055},
	{160,	1027},
	{180,	1000},
	{200,	973},
	{220,	948},
	{240,	923},
	{260,	899},
	{280,	876},
	{300,	854},
	{320,	832},
	{340,	812},
	{360,	792},
	{380,	774},
	{400,	756},
	{420,	739},
	{440,	723},
	{460,	707},
	{480,	692},
	{500,	679},
	{520,	665},
	{540,	653},
	{560,	641},
	{580,	630},
	{600,	619},
	{620,	609},
	{640,	600},
	{660,	591},
	{680,	583},
	{700,	575},
	{720,	567},
	{740,	560},
	{760,	553},
	{780,	547},
	{800,	541},
	{820,	535},
	{840,	530},
	{860,	524},
	{880,	520},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuc_btm_threshold[] = {
	{-200,	1539},
	{-180,	1515},
	{-160,	1491},
	{-140,	1465},
	{-120,	1438},
	{-100,	1410},
	{-80,	1381},
	{-60,	1352},
	{-40,	1322},
	{-20,	1291},
	{0,	1260},
	{20,	1229},
	{40,	1197},
	{60,	1166},
	{80,	1134},
	{100,	1103},
	{120,	1072},
	{140,	1042},
	{160,	1012},
	{180,	982},
	{200,	954},
	{220,	926},
	{240,	899},
	{260,	873},
	{280,	847},
	{300,	823},
	{320,	800},
	{340,	777},
	{360,	756},
	{380,	735},
	{400,	715},
	{420,	696},
	{440,	679},
	{460,	662},
	{480,	645},
	{500,	630},
	{520,	615},
	{540,	602},
	{560,	588},
	{580,	576},
	{600,	564},
	{620,	553},
	{640,	543},
	{660,	533},
	{680,	523},
	{700,	515},
	{720,	506},
	{740,	498},
	{760,	491},
	{780,	484},
	{800,	477},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skue_btm_threshold[] = {
	{-200,	1385},
	{-180,	1353},
	{-160,	1320},
	{-140,	1287},
	{-120,	1253},
	{-100,	1218},
	{-80,	1184},
	{-60,	1149},
	{-40,	1115},
	{-20,	1080},
	{0,	1046},
	{20,	1013},
	{40,	980},
	{60,	948},
	{80,	917},
	{100,	887},
	{120,	858},
	{140,	830},
	{160,	803},
	{180,	777},
	{200,	752},
	{220,	729},
	{240,	706},
	{260,	685},
	{280,	664},
	{300,	645},
	{320,	626},
	{340,	609},
	{360,	593},
	{380,	577},
	{400,	563},
	{420,	549},
	{440,	536},
	{460,	524},
	{480,	512},
	{500,	501},
	{520,	491},
	{540,	481},
	{560,	472},
	{580,	464},
	{600,	456},
	{620,	448},
	{640,	441},
	{660,	435},
	{680,	428},
	{700,	423},
	{720,	417},
	{740,	412},
	{760,	407},
	{780,	402},
	{800,	398},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skul_btm_threshold[] = {
	{-200,	1515},
	{-180,	1492},
	{-160,	1467},
	{-140,	1441},
	{-120,	1413},
	{-100,	1386},
	{-80,	1357},
	{-60,	1327},
	{-40,	1296},
	{-20,	1264},
	{0,	1232},
	{20,	1200},
	{40,	1167},
	{60,	1134},
	{80,	1100},
	{100,	1050},
	{120,	1034},
	{140,	1000},
	{160,	967},
	{180,	935},
	{200,	903},
	{220,	871},
	{240,	840},
	{260,	810},
	{280,	780},
	{300,	751},
	{320,	723},
	{340,	696},
	{360,	670},
	{380,	645},
	{400,	620},
	{420,	597},
	{440,	574},
	{460,	552},
	{480,	531},
	{500,	512},
	{520,	492},
	{540,	474},
	{560,	457},
	{580,	440},
	{600,	425},
	{620,	410},
	{640,	396},
	{660,	383},
	{680,	370},
	{700,	358},
	{720,	347},
	{740,	336},
	{760,	326},
	{780,	311},
	{800,	307},
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_100k_104ef_104fb[] = {
	{1758,	-40},
	{1742,	-35},
	{1719,	-30},
	{1691,	-25},
	{1654,	-20},
	{1608,	-15},
	{1551,	-10},
	{1483,	-5},
	{1404,	0},
	{1315,	5},
	{1218,	10},
	{1114,	15},
	{1007,	20},
	{900,	25},
	{795,	30},
	{696,	35},
	{605,	40},
	{522,	45},
	{448,	50},
	{383,	55},
	{327,	60},
	{278,	65},
	{237,	70},
	{202,	75},
	{172,	80},
	{146,	85},
	{125,	90},
	{107,	95},
	{92,	100},
	{79,	105},
	{68,	110},
	{59,	115},
	{51,	120},
	{44,	125}
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_150k_104ef_104fb[] = {
	{1738,	-40},
	{1714,	-35},
	{1682,	-30},
	{1641,	-25},
	{1589,	-20},
	{1526,	-15},
	{1451,	-10},
	{1363,	-5},
	{1266,	0},
	{1159,	5},
	{1048,	10},
	{936,	15},
	{825,	20},
	{720,	25},
	{622,	30},
	{533,	35},
	{454,	40},
	{385,	45},
	{326,	50},
	{275,	55},
	{232,	60},
	{195,	65},
	{165,	70},
	{139,	75},
	{118,	80},
	{100,	85},
	{85,	90},
	{73,	95},
	{62,	100},
	{53,	105},
	{46,	110},
	{40,	115},
	{34,	120},
	{30,	125}
};

static const struct qpnp_vadc_map_pt adcmap_smb_batt_therm[] = {
	{-300,	1625},
	{-200,	1515},
	{-100,	1368},
	{0,	1192},
	{10,	1173},
	{20,	1154},
	{30,	1135},
	{40,	1116},
	{50,	1097},
	{60,	1078},
	{70,	1059},
	{80,	1040},
	{90,	1020},
	{100,	1001},
	{110,	982},
	{120,	963},
	{130,	944},
	{140,	925},
	{150,	907},
	{160,	888},
	{170,	870},
	{180,	851},
	{190,	833},
	{200,	815},
	{210,	797},
	{220,	780},
	{230,	762},
	{240,	745},
	{250,	728},
	{260,	711},
	{270,	695},
	{280,	679},
	{290,	663},
	{300,	647},
	{310,	632},
	{320,	616},
	{330,	602},
	{340,	587},
	{350,	573},
	{360,	559},
	{370,	545},
	{380,	531},
	{390,	518},
	{400,	505},
	{410,	492},
	{420,	480},
	{430,	465},
	{440,	456},
	{450,	445},
	{460,	433},
	{470,	422},
	{480,	412},
	{490,	401},
	{500,	391},
	{510,	381},
	{520,	371},
	{530,	362},
	{540,	352},
	{550,	343},
	{560,	335},
	{570,	326},
	{580,	318},
	{590,	309},
	{600,	302},
	{610,	294},
	{620,	286},
	{630,	279},
	{640,	272},
	{650,	265},
	{660,	258},
	{670,	252},
	{680,	245},
	{690,	239},
	{700,	233},
	{710,	227},
	{720,	221},
	{730,	216},
	{740,	211},
	{750,	205},
	{760,	200},
	{770,	195},
	{780,	190},
	{790,	186}
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_ncp03wf683[] = {
	{1742,	-40},
	{1718,	-35},
	{1687,	-30},
	{1647,	-25},
	{1596,	-20},
	{1534,	-15},
	{1459,	-10},
	{1372,	-5},
	{1275,	0},
	{1169,	5},
	{1058,	10},
	{945,	15},
	{834,	20},
	{729,	25},
	{630,	30},
	{541,	35},
	{461,	40},
	{392,	45},
	{332,	50},
	{280,	55},
	{236,	60},
	{199,	65},
	{169,	70},
	{142,	75},
	{121,	80},
	{102,	85},
	{87,	90},
	{74,	95},
	{64,	100},
	{55,	105},
	{47,	110},
	{40,	115},
	{35,	120},
	{30,	125}
};

static int32_t qpnp_adc_map_voltage_temp(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
	}

	return 0;
}

static int32_t qpnp_adc_map_temp_voltage(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
	}

	return 0;
}

static int64_t qpnp_adc_scale_ratiometric_calib(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties)
{
	int64_t adc_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties)
		return -EINVAL;

	adc_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_RATIOMETRIC].adc_gnd)
		* adc_properties->adc_vdd_reference;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}
	do_div(adc_voltage,
		chan_properties->adc_graph[CALIB_RATIOMETRIC].dy);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return adc_voltage;
}

int32_t qpnp_adc_scale_pmic_therm(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t pmic_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result
		|| !chan_properties->adc_graph[CALIB_ABSOLUTE].dy)
		return -EINVAL;

	pmic_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[CALIB_ABSOLUTE].dx;
	if (pmic_voltage < 0) {
		negative_offset = 1;
		pmic_voltage = -pmic_voltage;
	}
	do_div(pmic_voltage,
		chan_properties->adc_graph[CALIB_ABSOLUTE].dy);
	if (negative_offset)
		pmic_voltage = -pmic_voltage;
	pmic_voltage += chan_properties->adc_graph[CALIB_ABSOLUTE].dx;

	if (pmic_voltage > 0) {
		/* 2mV/K */
		adc_chan_result->measurement = pmic_voltage*
			chan_properties->offset_gain_denominator;

		do_div(adc_chan_result->measurement,
			chan_properties->offset_gain_numerator * 2);
	} else {
		adc_chan_result->measurement = 0;
	}
	/* Change to .001 deg C */
	adc_chan_result->measurement -= KELVINMIL_DEGMIL;
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_pmic_therm);

int32_t qpnp_adc_scale_millidegc_pmic_voltage_thr(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0, sign = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_ABSOLUTE);
	if (rc < 0) {
		pr_err("Could not acquire gain and offset\n");
		return rc;
	}

	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	low_output = (param->low_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	low_output = (low_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (low_output < 0) {
		sign = 1;
		low_output = -low_output;
	}
	do_div(low_output, QPNP_ADC_625_UV);
	if (sign)
		low_output = -low_output;
	low_output += btm_param.adc_gnd;

	sign = 0;
	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	high_output = (param->high_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	high_output = (high_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (high_output < 0) {
		sign = 1;
		high_output = -high_output;
	}
	do_div(high_output, QPNP_ADC_625_UV);
	if (sign)
		high_output = -high_output;
	high_output += btm_param.adc_gnd;

	*low_threshold = (uint32_t) low_output;
	*high_threshold = (uint32_t) high_output;
	pr_debug("high_temp:%d, low_temp:%d\n", param->high_temp,
				param->low_temp);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_millidegc_pmic_voltage_thr);

/* Scales the ADC code to degC using the mapping
 * table for the XO thermistor.
 */
int32_t qpnp_adc_tdkntcg_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t xo_thm = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	xo_thm = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		xo_thm, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tdkntcg_therm);

int32_t qpnp_adc_scale_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;
/*[Arima_5830][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160525 begin*/
/*[Arima_5833][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160523 begin*/
#if defined(CONFIG_BSP_HW_SKU_5830) || defined(CONFIG_BSP_HW_SKU_5833)
	int rc = 0;
	enum board_version hw_version = 0;
#endif
/*[Arima_5833][bozhi_lin] 20160523 end*/
/*[Arima_5830][bozhi_lin] 20160525 end*/

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

/*[Arima_5830][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160525 begin*/
/*[Arima_5833][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160523 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
	rc = qpnp_get_vadc_hw_version(chip, &hw_version);
	if (rc < 0) {
		pr_err("[B]%s(%d): Could not get hw version, set to board_pr\n", __func__, __LINE__);
		hw_version = board_pr;
	}

	if (hw_version >= board_pr) {
		return qpnp_adc_map_temp_voltage(
				adcmap_btm_threshold_pr,
				ARRAY_SIZE(adcmap_btm_threshold_pr),
				bat_voltage,
				&adc_chan_result->physical);
	} else {
		return qpnp_adc_map_temp_voltage(
				adcmap_btm_threshold,
				ARRAY_SIZE(adcmap_btm_threshold),
				bat_voltage,
				&adc_chan_result->physical);
	}
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
	rc = qpnp_get_vadc_hw_version(chip, &hw_version);
	if (rc < 0) {
		pr_err("[B]%s(%d): Could not get hw version, set to board_pr\n", __func__, __LINE__);
		hw_version = board_pr;
	}

	if (hw_version >= board_er2) {
		return qpnp_adc_map_temp_voltage(
				adcmap_btm_threshold_er2,
				ARRAY_SIZE(adcmap_btm_threshold_er2),
				bat_voltage,
				&adc_chan_result->physical);
	} else {
		return qpnp_adc_map_temp_voltage(
				adcmap_btm_threshold,
				ARRAY_SIZE(adcmap_btm_threshold),
				bat_voltage,
				&adc_chan_result->physical);
	}
#else
	return qpnp_adc_map_temp_voltage(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
#endif
/*[Arima_5833][bozhi_lin] 20160523 end*/
/*[Arima_5830][bozhi_lin] 20160525 end*/
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_therm);

int32_t qpnp_adc_scale_qrd_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_batt_therm);

int32_t qpnp_adc_scale_qrd_skuaa_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuaa_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuaa_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuaa_batt_therm);

int32_t qpnp_adc_scale_qrd_skug_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skug_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skug_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skug_batt_therm);

int32_t qpnp_adc_scale_qrd_skuh_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuh_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuh_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuh_batt_therm);

int32_t qpnp_adc_scale_qrd_skuc_batt_therm(struct qpnp_vadc_chip *chip,
			int32_t adc_code,
			const struct qpnp_adc_properties *adc_properties,
			const struct qpnp_vadc_chan_properties *chan_properties,
			struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuc_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuc_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuc_batt_therm);

int32_t qpnp_adc_scale_qrd_skue_batt_therm(struct qpnp_vadc_chip *chip,
			int32_t adc_code,
			const struct qpnp_adc_properties *adc_properties,
			const struct qpnp_vadc_chan_properties *chan_properties,
			struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skue_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skue_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skue_batt_therm);

int32_t qpnp_adc_scale_qrd_skul_batt_therm(struct qpnp_vadc_chip *chip,
			int32_t adc_code,
			const struct qpnp_adc_properties *adc_properties,
			const struct qpnp_vadc_chan_properties *chan_properties,
			struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skul_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skul_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skul_batt_therm);

int32_t qpnp_adc_scale_smb_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_smb_batt_therm,
			ARRAY_SIZE(adcmap_smb_batt_therm),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_smb_batt_therm);

int32_t qpnp_adc_scale_therm_pu1(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_150k_104ef_104fb,
		ARRAY_SIZE(adcmap_150k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu1);

int32_t qpnp_adc_scale_therm_pu2(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu2);

int32_t qpnp_adc_tm_scale_voltage_therm_pu2(struct qpnp_vadc_chip *chip,
					uint32_t reg, int64_t *result)
{
	int64_t adc_voltage = 0;
	struct qpnp_vadc_linear_graph param1;
	int negative_offset;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	adc_voltage = (reg - param1.adc_gnd) * param1.adc_vref;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}

	do_div(adc_voltage, param1.dy);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		adc_voltage, result);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_voltage_therm_pu2);

int32_t qpnp_adc_tm_scale_therm_voltage_pu2(struct qpnp_vadc_chip *chip,
				struct qpnp_adc_tm_config *param)
{
	struct qpnp_vadc_linear_graph param1;
	int rc;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->low_thr_temp, &param->low_thr_voltage);
	if (rc)
		return rc;

	param->low_thr_voltage *= param1.dy;
	do_div(param->low_thr_voltage, param1.adc_vref);
	param->low_thr_voltage += param1.adc_gnd;

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->high_thr_temp, &param->high_thr_voltage);
	if (rc)
		return rc;

	param->high_thr_voltage *= param1.dy;
	do_div(param->high_thr_voltage, param1.adc_vref);
	param->high_thr_voltage += param1.adc_gnd;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_therm_voltage_pu2);

int32_t qpnp_adc_scale_therm_ncp03(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_ncp03wf683,
		ARRAY_SIZE(adcmap_ncp03wf683),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_ncp03);

int32_t qpnp_adc_scale_batt_id(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t batt_id_voltage = 0;

	batt_id_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	adc_chan_result->physical = batt_id_voltage;
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_id);

int32_t qpnp_adc_scale_default(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0, negative_offset = 0;
	int64_t scale_voltage = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	scale_voltage = (adc_code -
		chan_properties->adc_graph[chan_properties->calib_type].adc_gnd)
		* chan_properties->adc_graph[chan_properties->calib_type].dx;
	if (scale_voltage < 0) {
		negative_offset = 1;
		scale_voltage = -scale_voltage;
	}
	do_div(scale_voltage,
		chan_properties->adc_graph[chan_properties->calib_type].dy);
	if (negative_offset)
		scale_voltage = -scale_voltage;

	if (chan_properties->calib_type == CALIB_ABSOLUTE)
		scale_voltage +=
		chan_properties->adc_graph[chan_properties->calib_type].dx;
	else
		scale_voltage *= 1000;

	if (scale_voltage < 0) {
		if (adc_properties->bipolar) {
			scale_voltage = -scale_voltage;
			negative_rawfromoffset = 1;
		} else {
			scale_voltage = 0;
		}
	}

	adc_chan_result->measurement = scale_voltage *
				chan_properties->offset_gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement,
				chan_properties->offset_gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement = -adc_chan_result->measurement;

	/*
	 * Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input
	 */
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_default);

int32_t qpnp_adc_usb_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph usb_param;

	qpnp_get_vadc_gain_and_offset(chip, &usb_param, CALIB_RATIOMETRIC);

	*low_threshold = param->low_thr * usb_param.dy;
	do_div(*low_threshold, usb_param.adc_vref);
	*low_threshold += usb_param.adc_gnd;

	*high_threshold = param->high_thr * usb_param.dy;
	do_div(*high_threshold, usb_param.adc_vref);
	*high_threshold += usb_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_usb_scaler);

int32_t qpnp_adc_vbatt_rscaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_vbatt_rscaler);

int32_t qpnp_adc_absolute_rthr(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr) - QPNP_ADC_625_UV) * vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr) - QPNP_ADC_625_UV) * vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_absolute_rthr);

int32_t qpnp_vadc_absolute_rthr(struct qpnp_vadc_chip *chip,
		const struct qpnp_vadc_chan_properties *chan_prop,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	if (!chan_prop || !chan_prop->offset_gain_numerator ||
		!chan_prop->offset_gain_denominator)
		return -EINVAL;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr)/chan_prop->offset_gain_denominator
					- QPNP_ADC_625_UV) * vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	low_thr = low_thr * chan_prop->offset_gain_numerator;
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr)/chan_prop->offset_gain_denominator
					- QPNP_ADC_625_UV) * vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	high_thr = high_thr * chan_prop->offset_gain_numerator;
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_vadc_absolute_rthr);

int32_t qpnp_adc_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;
/*[Arima_5830][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160525 begin*/
/*[Arima_5833][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160523 begin*/
#if defined(CONFIG_BSP_HW_SKU_5830) || defined(CONFIG_BSP_HW_SKU_5833)
	enum board_version hw_version = 0;

	rc = qpnp_get_vadc_hw_version(chip, &hw_version);
	if (rc < 0) {
		pr_err("[B]%s(%d): Could not get hw version, set to board_pr\n", __func__, __LINE__);
		hw_version = board_pr;
	}
#endif
/*[Arima_5833][bozhi_lin] 20160523 end*/
/*[Arima_5830][bozhi_lin] 20160525 end*/

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
/*[Arima_5830][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160525 begin*/
/*[Arima_5833][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160523 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
	if (hw_version >= board_pr) {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_pr,
			ARRAY_SIZE(adcmap_btm_threshold_pr),
			(param->low_temp),
			&low_output);
	} else {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			(param->low_temp),
			&low_output);
	}
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
	if (hw_version >= board_er2) {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_er2,
			ARRAY_SIZE(adcmap_btm_threshold_er2),
			(param->low_temp),
			&low_output);
	} else {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			(param->low_temp),
			&low_output);
	}
#else
	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->low_temp),
		&low_output);
#endif
/*[Arima_5833][bozhi_lin] 20160523 end*/
/*[Arima_5830][bozhi_lin] 20160525 end*/
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

/*[Arima_5830][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160525 begin*/
/*[Arima_5833][bozhi_lin] dynamic apply battery thermal adc table based on PCBA_ID0 & PCBA_ID1 to check hw version 20160523 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
	if (hw_version >= board_pr) {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_pr,
			ARRAY_SIZE(adcmap_btm_threshold_pr),
			(param->high_temp),
			&high_output);
	} else {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			(param->high_temp),
			&high_output);
	}
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
	if (hw_version >= board_er2) {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_er2,
			ARRAY_SIZE(adcmap_btm_threshold_er2),
			(param->high_temp),
			&high_output);
	} else {
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			(param->high_temp),
			&high_output);
	}
#else
	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->high_temp),
		&high_output);
#endif
/*[Arima_5833][bozhi_lin] 20160523 end*/
/*[Arima_5830][bozhi_lin] 20160525 end*/
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_btm_scaler);

int32_t qpnp_adc_qrd_skuh_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
		adcmap_qrd_skuh_btm_threshold,
		ARRAY_SIZE(adcmap_qrd_skuh_btm_threshold),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
		adcmap_qrd_skuh_btm_threshold,
		ARRAY_SIZE(adcmap_qrd_skuh_btm_threshold),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_qrd_skuh_btm_scaler);

int32_t qpnp_adc_qrd_skue_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
		adcmap_qrd_skue_btm_threshold,
		ARRAY_SIZE(adcmap_qrd_skue_btm_threshold),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
		adcmap_qrd_skue_btm_threshold,
		ARRAY_SIZE(adcmap_qrd_skue_btm_threshold),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_qrd_skue_btm_scaler);

int32_t qpnp_adc_smb_btm_rscaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
		adcmap_smb_batt_therm,
		ARRAY_SIZE(adcmap_smb_batt_therm),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
		adcmap_smb_batt_therm,
		ARRAY_SIZE(adcmap_smb_batt_therm),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_smb_btm_rscaler);

int32_t qpnp_vadc_check_result(int32_t *data, bool recalib_check)
{
	if (recalib_check)
		return 0;

	if (*data < QPNP_VADC_MIN_ADC_CODE)
		*data = QPNP_VADC_MIN_ADC_CODE;
	else if (*data > QPNP_VADC_MAX_ADC_CODE)
		*data = QPNP_VADC_MAX_ADC_CODE;

	return 0;
}
EXPORT_SYMBOL(qpnp_vadc_check_result);

int qpnp_adc_get_revid_version(struct device *dev)
{
	struct pmic_revid_data *revid_data;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(dev->of_node,
						"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_debug("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	revid_data = get_revid_data(revid_dev_node);
	if (IS_ERR(revid_data)) {
		pr_debug("revid error rc = %ld\n", PTR_ERR(revid_data));
		return -EINVAL;
	}

	if ((revid_data->rev1 == PM8941_V3P1_REV1) &&
		(revid_data->rev2 == PM8941_V3P1_REV2) &&
		(revid_data->rev3 == PM8941_V3P1_REV3) &&
		(revid_data->rev4 == PM8941_V3P1_REV4) &&
		(revid_data->pmic_type == PM8941_V3P1_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P1_SUBTYPE))
			return QPNP_REV_ID_8941_3_1;
	else if ((revid_data->rev1 == PM8941_V3P0_REV1) &&
		(revid_data->rev2 == PM8941_V3P0_REV2) &&
		(revid_data->rev3 == PM8941_V3P0_REV3) &&
		(revid_data->rev4 == PM8941_V3P0_REV4) &&
		(revid_data->pmic_type == PM8941_V3P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P0_SUBTYPE))
			return QPNP_REV_ID_8941_3_0;
	else if ((revid_data->rev1 == PM8941_V2P0_REV1) &&
		(revid_data->rev2 == PM8941_V2P0_REV2) &&
		(revid_data->rev3 == PM8941_V2P0_REV3) &&
		(revid_data->rev4 == PM8941_V2P0_REV4) &&
		(revid_data->pmic_type == PM8941_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V2P0_SUBTYPE))
			return QPNP_REV_ID_8941_2_0;
	else if ((revid_data->rev1 == PM8226_V2P2_REV1) &&
		(revid_data->rev2 == PM8226_V2P2_REV2) &&
		(revid_data->rev3 == PM8226_V2P2_REV3) &&
		(revid_data->rev4 == PM8226_V2P2_REV4) &&
		(revid_data->pmic_type == PM8226_V2P2_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P2_SUBTYPE))
			return QPNP_REV_ID_8026_2_2;
	else if ((revid_data->rev1 == PM8226_V2P1_REV1) &&
		(revid_data->rev2 == PM8226_V2P1_REV2) &&
		(revid_data->rev3 == PM8226_V2P1_REV3) &&
		(revid_data->rev4 == PM8226_V2P1_REV4) &&
		(revid_data->pmic_type == PM8226_V2P1_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P1_SUBTYPE))
			return QPNP_REV_ID_8026_2_1;
	else if ((revid_data->rev1 == PM8226_V2P0_REV1) &&
		(revid_data->rev2 == PM8226_V2P0_REV2) &&
		(revid_data->rev3 == PM8226_V2P0_REV3) &&
		(revid_data->rev4 == PM8226_V2P0_REV4) &&
		(revid_data->pmic_type == PM8226_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P0_SUBTYPE))
			return QPNP_REV_ID_8026_2_0;
	else if ((revid_data->rev1 == PM8226_V1P0_REV1) &&
		(revid_data->rev2 == PM8226_V1P0_REV2) &&
		(revid_data->rev3 == PM8226_V1P0_REV3) &&
		(revid_data->rev4 == PM8226_V1P0_REV4) &&
		(revid_data->pmic_type == PM8226_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V1P0_SUBTYPE))
			return QPNP_REV_ID_8026_1_0;
	else if ((revid_data->rev1 == PM8110_V1P0_REV1) &&
		(revid_data->rev2 == PM8110_V1P0_REV2) &&
		(revid_data->rev3 == PM8110_V1P0_REV3) &&
		(revid_data->rev4 == PM8110_V1P0_REV4) &&
		(revid_data->pmic_type == PM8110_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V1P0_SUBTYPE))
			return QPNP_REV_ID_8110_1_0;
	else if ((revid_data->rev1 == PM8110_V2P0_REV1) &&
		(revid_data->rev2 == PM8110_V2P0_REV2) &&
		(revid_data->rev3 == PM8110_V2P0_REV3) &&
		(revid_data->rev4 == PM8110_V2P0_REV4) &&
		(revid_data->pmic_type == PM8110_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V2P0_SUBTYPE))
			return QPNP_REV_ID_8110_2_0;
	else if ((revid_data->rev1 == PM8916_V1P0_REV1) &&
		(revid_data->rev2 == PM8916_V1P0_REV2) &&
		(revid_data->rev3 == PM8916_V1P0_REV3) &&
		(revid_data->rev4 == PM8916_V1P0_REV4) &&
		(revid_data->pmic_type == PM8916_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8916_V1P0_SUBTYPE))
			return QPNP_REV_ID_8916_1_0;
	else if ((revid_data->rev1 == PM8916_V1P1_REV1) &&
		(revid_data->rev2 == PM8916_V1P1_REV2) &&
		(revid_data->rev3 == PM8916_V1P1_REV3) &&
		(revid_data->rev4 == PM8916_V1P1_REV4) &&
		(revid_data->pmic_type == PM8916_V1P1_TYPE) &&
		(revid_data->pmic_subtype == PM8916_V1P1_SUBTYPE))
			return QPNP_REV_ID_8916_1_1;
	else if ((revid_data->rev1 == PM8916_V2P0_REV1) &&
		(revid_data->rev2 == PM8916_V2P0_REV2) &&
		(revid_data->rev3 == PM8916_V2P0_REV3) &&
		(revid_data->rev4 == PM8916_V2P0_REV4) &&
		(revid_data->pmic_type == PM8916_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8916_V2P0_SUBTYPE))
			return QPNP_REV_ID_8916_2_0;
	else if ((revid_data->rev1 == PM8909_V1P0_REV1) &&
		(revid_data->rev2 == PM8909_V1P0_REV2) &&
		(revid_data->rev3 == PM8909_V1P0_REV3) &&
		(revid_data->rev4 == PM8909_V1P0_REV4) &&
		(revid_data->pmic_type == PM8909_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8909_V1P0_SUBTYPE))
			return QPNP_REV_ID_8909_1_0;
	else if ((revid_data->rev1 == PM8909_V1P1_REV1) &&
		(revid_data->rev2 == PM8909_V1P1_REV2) &&
		(revid_data->rev3 == PM8909_V1P1_REV3) &&
		(revid_data->rev4 == PM8909_V1P1_REV4) &&
		(revid_data->pmic_type == PM8909_V1P1_TYPE) &&
		(revid_data->pmic_subtype == PM8909_V1P1_SUBTYPE))
			return QPNP_REV_ID_8909_1_1;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(qpnp_adc_get_revid_version);

int32_t qpnp_adc_get_devicetree_data(struct spmi_device *spmi,
			struct qpnp_adc_drv *adc_qpnp)
{
	struct device_node *node = spmi->dev.of_node;
	struct resource *res;
	struct device_node *child;
	struct qpnp_adc_amux *adc_channel_list;
	struct qpnp_adc_properties *adc_prop;
	struct qpnp_adc_amux_properties *amux_prop;
	int count_adc_channel_list = 0, decimation, rc = 0, i = 0;

	if (!node)
		return -EINVAL;

	for_each_child_of_node(node, child)
		count_adc_channel_list++;

	if (!count_adc_channel_list) {
		pr_err("No channel listing\n");
		return -EINVAL;
	}

	adc_qpnp->spmi = spmi;

	adc_prop = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_adc_properties),
					GFP_KERNEL);
	if (!adc_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	adc_channel_list = devm_kzalloc(&spmi->dev,
		((sizeof(struct qpnp_adc_amux)) * count_adc_channel_list),
				GFP_KERNEL);
	if (!adc_channel_list) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	amux_prop = devm_kzalloc(&spmi->dev,
		sizeof(struct qpnp_adc_amux_properties) +
		sizeof(struct qpnp_vadc_chan_properties), GFP_KERNEL);
	if (!amux_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	adc_qpnp->adc_channels = adc_channel_list;
	adc_qpnp->amux_prop = amux_prop;

	for_each_child_of_node(node, child) {
		int channel_num, scaling, post_scaling, hw_settle_time;
		int fast_avg_setup, calib_type = 0, rc;
		const char *calibration_param, *channel_name;

		channel_name = of_get_property(child,
				"label", NULL) ? : child->name;
		if (!channel_name) {
			pr_err("Invalid channel name\n");
			return -EINVAL;
		}

		rc = of_property_read_u32(child, "reg", &channel_num);
		if (rc) {
			pr_err("Invalid channel num\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child, "qcom,decimation",
								&decimation);
		if (rc) {
			pr_err("Invalid channel decimation property\n");
			return -EINVAL;
		}
		if (!of_device_is_compatible(node, "qcom,qpnp-iadc")) {
			rc = of_property_read_u32(child,
				"qcom,hw-settle-time", &hw_settle_time);
			if (rc) {
				pr_err("Invalid channel hw settle time property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,pre-div-channel-scaling", &scaling);
			if (rc) {
				pr_err("Invalid channel scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,scale-function", &post_scaling);
			if (rc) {
				pr_err("Invalid channel post scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_string(child,
				"qcom,calibration-type", &calibration_param);
			if (rc) {
				pr_err("Invalid calibration type\n");
				return -EINVAL;
			}
			if (!strcmp(calibration_param, "absolute"))
				calib_type = CALIB_ABSOLUTE;
			else if (!strcmp(calibration_param, "ratiometric"))
				calib_type = CALIB_RATIOMETRIC;
			else {
				pr_err("%s: Invalid calibration property\n",
						__func__);
				return -EINVAL;
			}
		}
		rc = of_property_read_u32(child,
				"qcom,fast-avg-setup", &fast_avg_setup);
		if (rc) {
			pr_err("Invalid channel fast average setup\n");
			return -EINVAL;
		}
		/* Individual channel properties */
		adc_channel_list[i].name = (char *)channel_name;
		adc_channel_list[i].channel_num = channel_num;
		adc_channel_list[i].adc_decimation = decimation;
		adc_channel_list[i].fast_avg_setup = fast_avg_setup;
		if (!of_device_is_compatible(node, "qcom,qpnp-iadc")) {
			adc_channel_list[i].chan_path_prescaling = scaling;
			adc_channel_list[i].adc_scale_fn = post_scaling;
			adc_channel_list[i].hw_settle_time = hw_settle_time;
			adc_channel_list[i].calib_type = calib_type;
		}
		i++;
	}

	/* Get the ADC VDD reference voltage and ADC bit resolution */
	rc = of_property_read_u32(node, "qcom,adc-vdd-reference",
			&adc_prop->adc_vdd_reference);
	if (rc) {
		pr_err("Invalid adc vdd reference property\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,adc-bit-resolution",
			&adc_prop->bitresolution);
	if (rc) {
		pr_err("Invalid adc bit resolution property\n");
		return -EINVAL;
	}
	adc_qpnp->adc_prop = adc_prop;

	/* Get the peripheral address */
	res = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("No base address definition\n");
		return -EINVAL;
	}

	adc_qpnp->slave = spmi->sid;
	adc_qpnp->offset = res->start;

	/* Register the ADC peripheral interrupt */
	adc_qpnp->adc_irq_eoc = spmi_get_irq_byname(spmi, NULL,
						"eoc-int-en-set");
	if (adc_qpnp->adc_irq_eoc < 0) {
		pr_err("Invalid irq\n");
		return -ENXIO;
	}

	init_completion(&adc_qpnp->adc_rslt_completion);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_get_devicetree_data);
