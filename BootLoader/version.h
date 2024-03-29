/*
 * version.h
 *
 *  Created on: Mar 7, 2016
 *      Author: abid.esmail
 */

#ifndef _VERSION_H_
#define _VERSION_H_

/* 	IMPORTENT!
	Version entries rules (importent for parsing version number with external tools):
	- each part of version starts from '0x'
	- after each part of version additional characters are not allowed (like comments etc.)
	- the maximum length of each part of version is 2 digits (0xFF)
*/
/* 0xA : Application, 0xB: Bootloader */
#define FW_VER_BTLD_OR_APP 0xB
#define FW_VER_MAJOR 0x0
#define FW_VER_MINOR 0x1
#define FW_VER_BUILD 0x0

#endif /* _VERSION_H_ */
