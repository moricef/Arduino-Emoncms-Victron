/*
 * config.h config the keywords for Victron.DIRECT
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define MPPT_150_35
#ifdef MPPT_150_35

const byte buffsize = 32;
const byte value_bytes = 33;
const byte label_bytes = 9;
const byte num_keywords1 = 17;

char keywords1[num_keywords1][label_bytes] = { "PID", "FW", "SER#", "V", "I",
		"VPV", "PPV", "CS", "ERR", "LOAD", "H19", "H20", "H21", "H22", "H23",
		"HSDS", "Checksum" };

#define PID 0
#define FW 1
#define SER 2	// Offically SER# but # does not play that well as macro
#define V 3     // ScV
#define I 4     // ScI
#define VPV 5   // PVV
#define PPV 6   // PVI = PVV / VPV
#define CS 7    // ScS
#define ERR 8   // ScERR
#define LOAD 9  // SLs
#define H19 10
#define H20 11
#define H21 12
#define H22 13
#define H23 14
#define HSDS 15
#define CHECKSUM 16
#endif

#define INVERTER_24V_500VA
#ifdef INVERTER_24V_500VA

const byte num_keywords2 = 11;

char keywords2[num_keywords2][label_bytes] = { "PID", "FW", "SER#", "MODE",
		"CS", "AC_OUT_V", "AC_OUT_I", "V", "AR", "WARN", "Checksum" };

#define PID 0
#define FW 1
#define SER 2 // Offically SER# but # does not play that well as macro
#define MODE 3
#define CS 4
#define AC_OUT_V 5
#define AC_OUT_V 6
#define V 7
#define AR  8
#define WARN  9
#define Checksum10
#endif

#define BMV_700
#ifdef BMV_700

const byte num_keywords3 = 28;

char keywords3[num_keywords3][label_bytes] = { "PID", "V", "I", "P", "CE",
		"SOC", "TTG", "Alarm", "Relay", "AR", "BMV", "FW ", "Checksum", "H1",
		"H2", "H3", "H4", "H5", "H6", "H7", "H8", "H9", "H10", "H11", "H12",
		"H17", "H18", "Checksum" };

#define PID    0
#define V      1
#define I      2
#define P      3
#define CE     4
#define SOC    5
#define TTG    6
#define Alarm  7
#define Relay  8
#define AR     9
#define BMV    10
#define FW     11
#define Checksum 12
#define H1     13
#define H2     14
#define H3     15
#define H4     16
#define H5     17
#define H6     18
#define H7     19
#define H8     20
#define H9     21
#define H10    22
#define H11    23
#define H12    24
#define H17    25
#define H18    26
#define Checksum  27
#endif

#endif /* CONFIG_H_ */
