// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2007,2008 by Christopher Kilgour                        *
 *   techie |_at_| whiterocker |_dot_| com                                 *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

/* ----------------------------------------------------------------------
 *                      Internal Support, Helpers
 * ---------------------------------------------------------------------- */

struct tms470_flash_bank {
	unsigned ordinal;

	/* device identification register */
	uint32_t device_ident_reg;
    uint32_t version;
    uint32_t ram_ecc;
    uint32_t flash_ecc;
    uint32_t periph_parity;
    uint32_t io_voltage;
    uint32_t tech;
    uint32_t unique_id;
	const char *part_name;
};

#define TMS470R1A384_BANK2_NUM_SECTORS \
	ARRAY_SIZE(tms470r1a384_bank2_sectors)

static const struct flash_sector rm57l843_bank0_sectors[] = {
	{0x00000000, 0x00004000, -1, -1},
	{0x00004000, 0x00004000, -1, -1},
	{0x00008000, 0x00004000, -1, -1},
	{0x0000C000, 0x00004000, -1, -1},
	{0x00010000, 0x00004000, -1, -1},
	{0x00014000, 0x00004000, -1, -1},
	{0x00018000, 0x00004000, -1, -1},
	{0x00020000, 0x00020000, -1, -1},
	{0x00040000, 0x00020000, -1, -1},
	{0x00060000, 0x00020000, -1, -1},
	{0x00080000, 0x00040000, -1, -1},
	{0x000C0000, 0x00040000, -1, -1},
	{0x00100000, 0x00040000, -1, -1},
	{0x00140000, 0x00040000, -1, -1},
	{0x00180000, 0x00040000, -1, -1},
	{0x001C0000, 0x00040000, -1, -1},
};

#define RM57L843_BANK0_NUM_SECTORS \
	ARRAY_SIZE(rm57l843_bank0_sectors)

static const struct flash_sector rm57l843_bank1_sectors[] = {
	{0x00200000, 0x00020000, -1, -1},
	{0x00220000, 0x00020000, -1, -1},
	{0x00240000, 0x00020000, -1, -1},
	{0x00260000, 0x00020000, -1, -1},
	{0x00280000, 0x00020000, -1, -1},
	{0x002A0000, 0x00020000, -1, -1},
	{0x002C0000, 0x00020000, -1, -1},
	{0x002E0000, 0x00020000, -1, -1},
	{0x00300000, 0x00020000, -1, -1},
	{0x00320000, 0x00020000, -1, -1},
	{0x00340000, 0x00020000, -1, -1},
	{0x00360000, 0x00020000, -1, -1},
	{0x00380000, 0x00020000, -1, -1},
	{0x003A0000, 0x00020000, -1, -1},
	{0x003C0000, 0x00020000, -1, -1},
	{0x003E0000, 0x00020000, -1, -1},
};

#define RM57L843_BANK1_NUM_SECTORS \
	ARRAY_SIZE(rm57l843_bank1_sectors)

static const struct flash_sector rm57l843_bank7_sectors[] = {
	{0xF0200000, 0x00001000, -1, -1},
	{0xF0201000, 0x00001000, -1, -1},
	{0xF0202000, 0x00001000, -1, -1},
	{0xF0203000, 0x00001000, -1, -1},
	{0xF0204000, 0x00001000, -1, -1},
	{0xF0205000, 0x00001000, -1, -1},
	{0xF0206000, 0x00001000, -1, -1},
	{0xF0207000, 0x00001000, -1, -1},
	{0xF0208000, 0x00001000, -1, -1},
	{0xF0209000, 0x00001000, -1, -1},
	{0xF020A000, 0x00001000, -1, -1},
	{0xF020B000, 0x00001000, -1, -1},
	{0xF020C000, 0x00001000, -1, -1},
	{0xF020D000, 0x00001000, -1, -1},
	{0xF020E000, 0x00001000, -1, -1},
	{0xF020F000, 0x00001000, -1, -1},
	{0xF0210000, 0x00001000, -1, -1},
	{0xF0211000, 0x00001000, -1, -1},
	{0xF0212000, 0x00001000, -1, -1},
	{0xF0213000, 0x00001000, -1, -1},
	{0xF0214000, 0x00001000, -1, -1},
	{0xF0215000, 0x00001000, -1, -1},
	{0xF0216000, 0x00001000, -1, -1},
	{0xF0217000, 0x00001000, -1, -1},
	{0xF0218000, 0x00001000, -1, -1},
	{0xF0219000, 0x00001000, -1, -1},
	{0xF021A000, 0x00001000, -1, -1},
	{0xF021B000, 0x00001000, -1, -1},
	{0xF021C000, 0x00001000, -1, -1},
	{0xF021D000, 0x00001000, -1, -1},
	{0xF021E000, 0x00001000, -1, -1},
	{0xF021F000, 0x00001000, -1, -1},
};

#define RM57L843_BANK7_NUM_SECTORS \
	ARRAY_SIZE(rm57l843_bank7_sectors)


/* ---------------------------------------------------------------------- */

/**
 * @brief Reads the 'Device Identification Code Register' of a RM57L843 
 * microcontroller
 * 
 * @see https://www.ti.com/lit/ds/symlink/rm57l843.pdf chapter 9.6
 * @see https://www.ti.com/lit/ug/spnu562a/spnu562a.pdf chapter 2.5.1.46
 * 
 * @param bank 
 * @return int 
 */
static int rm57l8xx_read_part_info(struct flash_bank *bank)
{
	struct tms470_flash_bank *rm57l8xx_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t device_ident_reg;
	const char *part_name;

	/* we shall not rely on the caller in this test, this function allocates memory,
	   thus and executing the code more than once may cause memory leak */
	if (rm57l8xx_info->device_ident_reg)
		return ERROR_OK;

	/* read and parse the device identification register */
	target_read_u32(target, 0xFFFFFFF0, &device_ident_reg);

	LOG_INFO("device_ident_reg = 0x%08" PRIx32 "", device_ident_reg);

	if ((device_ident_reg & 7) != 0b101) {
		LOG_WARNING("Cannot identify target as a RM57L8xx family.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

    uint32_t version = (device_ident_reg >> 3) & 0x1f;
    uint32_t ram_ecc = (device_ident_reg >> 8) & 1;
    uint32_t flash_ecc = (device_ident_reg >> 9) & 0x3;
    uint32_t periph_parity = (device_ident_reg >> 11) & 1;
    uint32_t io_voltage = (device_ident_reg >> 12) & 1;
    uint32_t tech = (device_ident_reg >> 13) & 0xf;
    uint32_t unique_id = (device_ident_reg >> 17) & 0x3fff;

	free(bank->sectors);
	bank->sectors = NULL;
	bank->num_sectors = 0;

	/*
	 * If the part number is known, determine if the flash bank is valid
	 * based on the base address being within the known flash bank
	 * ranges.  Then fixup/complete the remaining fields of the flash
	 * bank structure.
	 */
	switch (device_ident_reg) {
		case 0x8044AD05:
			part_name = "RM57L843 (Rev A)";
			break;
		case 0x8044AD0D:
			part_name = "RM57L843 (Rev B)";
			break;
		default:
			LOG_WARNING("Could not identify revision 0x%02x as a member of the "
				"RM57L843 family.", (unsigned)version);
			return ERROR_FLASH_OPERATION_FAILED;
	}

	if (bank->base < 0x00200000) {
		rm57l8xx_info->ordinal = 0;
		bank->base = 0x00000000;
		bank->size = 2 * 1024 * 1024;
		bank->num_sectors = RM57L843_BANK0_NUM_SECTORS;
		bank->sectors = malloc(sizeof(rm57l843_bank0_sectors));

		if (!bank->sectors) { 
			return ERROR_FLASH_OPERATION_FAILED; 
		}

		(void) memcpy(bank->sectors, rm57l843_bank0_sectors,
			sizeof(rm57l843_bank0_sectors));
	} else if (bank->base < 0x00400000) {
		rm57l8xx_info->ordinal = 1;
		bank->base = 0x00400000;
		bank->size = 2 * 1024 * 1024;
		bank->num_sectors = RM57L843_BANK1_NUM_SECTORS;
		bank->sectors = malloc(sizeof(rm57l843_bank1_sectors));

		if (!bank->sectors) {
			return ERROR_FLASH_OPERATION_FAILED;
		}

		(void) memcpy(bank->sectors, rm57l843_bank1_sectors,
			sizeof(rm57l843_bank1_sectors));
	} else if (bank->base >= 0xF0200000 && bank->base < 0xF0220000) {
		rm57l8xx_info->ordinal = 7;
		bank->base = 0xF0200000;
		bank->size = 128 * 1024;
		bank->num_sectors = RM57L843_BANK7_NUM_SECTORS;
		bank->sectors = malloc(sizeof(rm57l843_bank7_sectors));

		if (!bank->sectors) {
			return ERROR_FLASH_OPERATION_FAILED;
		}

		(void) memcpy(bank->sectors, rm57l843_bank7_sectors,
			sizeof(rm57l843_bank7_sectors));
	} else {
		LOG_ERROR("No %s flash bank contains base address "
				TARGET_ADDR_FMT ".",
				part_name,
				bank->base);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* turn off memory selects */
	target_write_u32(target, 0xFFFFFFE4, 0x00000000); // Clear Reset status
	target_write_u32(target, 0xFFFFFFE0, 0x00000000); // Reset

    LOG_INFO("Identified %s, ver=%d, ram_ecc=%d, flash_ecc=%d, "
		"periph_parity=%d, io_voltage=%d, tech=%d, unique_id=%d",
		part_name, version, ram_ecc, flash_ecc, periph_parity,
		io_voltage, tech, unique_id);

	rm57l8xx_info->device_ident_reg = device_ident_reg;
    rm57l8xx_info->version = version;
    rm57l8xx_info->ram_ecc = ram_ecc;
    rm57l8xx_info->flash_ecc = flash_ecc;
    rm57l8xx_info->periph_parity = periph_parity;
    rm57l8xx_info->io_voltage = io_voltage;
    rm57l8xx_info->tech = tech;
    rm57l8xx_info->unique_id = unique_id;
	rm57l8xx_info->part_name = part_name;

	/*
	 * Disable reset on address access violation.
	 */
	target_write_u32(target, 0xFFFFFFE0, 0x00004007);

	return ERROR_OK;
}

/* ---------------------------------------------------------------------- */

static uint32_t keys_set;
static uint32_t flash_keys[4];

COMMAND_HANDLER(tms470_handle_flash_keyset_command)
{
	if (CMD_ARGC > 4)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (CMD_ARGC == 4) {
		int i;

		for (i = 0; i < 4; i++) {
			int start = (strncmp(CMD_ARGV[i], "0x", 2) == 0) ? 2 : 0;

			if (sscanf(&CMD_ARGV[i][start], "%" SCNx32 "", &flash_keys[i]) != 1) {
				command_print(CMD, "could not process flash key %s",
					CMD_ARGV[i]);
				LOG_ERROR("could not process flash key %s", CMD_ARGV[i]);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		}

		keys_set = 1;
	} else if (CMD_ARGC != 0) {
		command_print(CMD, "tms470 flash_keyset <key0> <key1> <key2> <key3>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (keys_set) {
		command_print(CMD,
			"using flash keys 0x%08" PRIx32 ", 0x%08" PRIx32 ", 0x%08" PRIx32 ", 0x%08" PRIx32 "",
			flash_keys[0],
			flash_keys[1],
			flash_keys[2],
			flash_keys[3]);
	} else
		command_print(CMD, "flash keys not set");

	return ERROR_OK;
}

static const uint32_t flash_keys_all_ones[] = { 0xFFFFFFFF, 0xFFFFFFFF,
		0xFFFFFFFF, 0xFFFFFFFF,};

static const uint32_t flash_keys_all_zeros[] = { 0x00000000, 0x00000000,
		0x00000000, 0x00000000,};

static const uint32_t flash_keys_mix1[] = { 0xf0fff0ff, 0xf0fff0ff,
		0xf0fff0ff, 0xf0fff0ff};

static const uint32_t flash_keys_mix2[] = { 0x0000ffff, 0x0000ffff,
		0x0000ffff, 0x0000ffff};

/* ---------------------------------------------------------------------- */

static int osc_mhz = 12;

COMMAND_HANDLER(tms470_handle_osc_megahertz_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (CMD_ARGC == 1)
		sscanf(CMD_ARGV[0], "%d", &osc_mhz);

	if (osc_mhz <= 0) {
		LOG_ERROR("osc_megahertz must be positive and non-zero!");
		command_print(CMD, "osc_megahertz must be positive and non-zero!");
		osc_mhz = 12;
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "osc_megahertz=%d", osc_mhz);

	return ERROR_OK;
}

/* ---------------------------------------------------------------------- */

static int plldis;

COMMAND_HANDLER(tms470_handle_plldis_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (CMD_ARGC == 1) {
		sscanf(CMD_ARGV[0], "%d", &plldis);
		plldis = plldis ? 1 : 0;
	}

	command_print(CMD, "plldis=%d", plldis);

	return ERROR_OK;
}

/* ---------------------------------------------------------------------- */

static int tms470_check_flash_unlocked(struct target *target)
{
	uint32_t fmbbusy;

	target_read_u32(target, 0xFFE89C08, &fmbbusy);
	LOG_INFO("tms470 fmbbusy = 0x%08" PRIx32 " -> %s",
		fmbbusy,
		fmbbusy & 0x8000 ? "unlocked" : "LOCKED");
	return fmbbusy & 0x8000 ? ERROR_OK : ERROR_FLASH_OPERATION_FAILED;
}

/* ---------------------------------------------------------------------- */

static int tms470_try_flash_keys(struct target *target, const uint32_t *key_set)
{
	uint32_t glbctrl, fmmstat;
	int retval = ERROR_FLASH_OPERATION_FAILED;

	/* set GLBCTRL.4  */
	target_read_u32(target, 0xFFFFFFDC, &glbctrl);
	target_write_u32(target, 0xFFFFFFDC, glbctrl | 0x10);

	/* only perform the key match when 3VSTAT is clear */
	target_read_u32(target, 0xFFE8BC0C, &fmmstat);
	if (!(fmmstat & 0x08)) {
		unsigned i;
		uint32_t fmbptr, fmbac2, orig_fmregopt;

		target_write_u32(target, 0xFFE8BC04, fmmstat & ~0x07);

		/* wait for pump ready */
		do {
			target_read_u32(target, 0xFFE8A814, &fmbptr);
			alive_sleep(1);
		} while (!(fmbptr & 0x0200));

		/* force max wait states */
		target_read_u32(target, 0xFFE88004, &fmbac2);
		target_write_u32(target, 0xFFE88004, fmbac2 | 0xff);

		/* save current access mode, force normal read mode */
		target_read_u32(target, 0xFFE89C00, &orig_fmregopt);
		target_write_u32(target, 0xFFE89C00, 0x00);

		for (i = 0; i < 4; i++) {
			uint32_t tmp;

			/* There is no point displaying the value of tmp, it is
			 * filtered by the chip.  The purpose of this read is to
			 * prime the unlocking logic rather than read out the value.
			 */
			target_read_u32(target, 0x00001FF0 + 4 * i, &tmp);

			LOG_INFO("tms470 writing fmpkey = 0x%08" PRIx32 "", key_set[i]);
			target_write_u32(target, 0xFFE89C0C, key_set[i]);
		}

		if (tms470_check_flash_unlocked(target) == ERROR_OK) {
			/*
			 * There seems to be a side-effect of reading the FMPKEY
			 * register in that it re-enables the protection.  So we
			 * re-enable it.
			 */
			for (i = 0; i < 4; i++) {
				uint32_t tmp;

				target_read_u32(target, 0x00001FF0 + 4 * i, &tmp);
				target_write_u32(target, 0xFFE89C0C, key_set[i]);
			}
			retval = ERROR_OK;
		}

		/* restore settings */
		target_write_u32(target, 0xFFE89C00, orig_fmregopt);
		target_write_u32(target, 0xFFE88004, fmbac2);
	}

	/* clear config bit */
	target_write_u32(target, 0xFFFFFFDC, glbctrl);

	return retval;
}

/* ---------------------------------------------------------------------- */

static int tms470_unlock_flash(struct flash_bank *bank)
{
	struct target *target = bank->target;
	const uint32_t *p_key_sets[5];
	unsigned i, key_set_count;

	if (keys_set) {
		key_set_count = 5;
		p_key_sets[0] = flash_keys;
		p_key_sets[1] = flash_keys_all_ones;
		p_key_sets[2] = flash_keys_all_zeros;
		p_key_sets[3] = flash_keys_mix1;
		p_key_sets[4] = flash_keys_mix2;
	} else {
		key_set_count = 4;
		p_key_sets[0] = flash_keys_all_ones;
		p_key_sets[1] = flash_keys_all_zeros;
		p_key_sets[2] = flash_keys_mix1;
		p_key_sets[3] = flash_keys_mix2;
	}

	for (i = 0; i < key_set_count; i++) {
		if (tms470_try_flash_keys(target, p_key_sets[i]) == ERROR_OK) {
			LOG_INFO("tms470 flash is unlocked");
			return ERROR_OK;
		}
	}

	LOG_WARNING("tms470 could not unlock flash memory protection level 2");
	return ERROR_FLASH_OPERATION_FAILED;
}

/* ---------------------------------------------------------------------- */

static int tms470_flash_initialize_internal_state_machine(struct flash_bank *bank)
{
	uint32_t fmmac2, fmmac1, fmmaxep, k, delay, glbctrl, sysclk;
	struct target *target = bank->target;
	struct tms470_flash_bank *tms470_info = bank->driver_priv;
	int result = ERROR_OK;

	/*
	 * Select the desired bank to be programmed by writing BANK[2:0] of
	 * FMMAC2.
	 */
	target_read_u32(target, 0xFFE8BC04, &fmmac2);
	fmmac2 &= ~0x0007;
	fmmac2 |= (tms470_info->ordinal & 7);
	target_write_u32(target, 0xFFE8BC04, fmmac2);
	LOG_DEBUG("set fmmac2 = 0x%04" PRIx32 "", fmmac2);

	/*
	 * Disable level 1 sector protection by setting bit 15 of FMMAC1.
	 */
	target_read_u32(target, 0xFFE8BC00, &fmmac1);
	fmmac1 |= 0x8000;
	target_write_u32(target, 0xFFE8BC00, fmmac1);
	LOG_DEBUG("set fmmac1 = 0x%04" PRIx32 "", fmmac1);

	/*
	 * FMTCREG = 0x2fc0;
	 */
	target_write_u32(target, 0xFFE8BC10, 0x2fc0);
	LOG_DEBUG("set fmtcreg = 0x2fc0");

	/*
	 * MAXPP = 50
	 */
	target_write_u32(target, 0xFFE8A07C, 50);
	LOG_DEBUG("set fmmaxpp = 50");

	/*
	 * MAXCP = 0xf000 + 2000
	 */
	target_write_u32(target, 0xFFE8A084, 0xf000 + 2000);
	LOG_DEBUG("set fmmaxcp = 0x%04x", 0xf000 + 2000);

	/*
	 * configure VHV
	 */
	target_read_u32(target, 0xFFE8A080, &fmmaxep);
	if (fmmaxep == 0xf000) {
		fmmaxep = 0xf000 + 4095;
		target_write_u32(target, 0xFFE8A80C, 0x9964);
		LOG_DEBUG("set fmptr3 = 0x9964");
	} else {
		fmmaxep = 0xa000 + 4095;
		target_write_u32(target, 0xFFE8A80C, 0x9b64);
		LOG_DEBUG("set fmptr3 = 0x9b64");
	}
	target_write_u32(target, 0xFFE8A080, fmmaxep);
	LOG_DEBUG("set fmmaxep = 0x%04" PRIx32 "", fmmaxep);

	/*
	 * FMPTR4 = 0xa000
	 */
	target_write_u32(target, 0xFFE8A810, 0xa000);
	LOG_DEBUG("set fmptr4 = 0xa000");

	/*
	 * FMPESETUP, delay parameter selected based on clock frequency.
	 *
	 * According to the TI App Note SPNU257 and flashing code, delay is
	 * int((sysclk(MHz) + 1) / 2), with a minimum of 5.  The system
	 * clock is usually derived from the ZPLL module, and selected by
	 * the plldis global.
	 */
	target_read_u32(target, 0xFFFFFFDC, &glbctrl);
	sysclk = (plldis ? 1 : (glbctrl & 0x08) ? 4 : 8) * osc_mhz / (1 + (glbctrl & 7));
	delay = (sysclk > 10) ? (sysclk + 1) / 2 : 5;
	target_write_u32(target, 0xFFE8A018, (delay << 4) | (delay << 8));
	LOG_DEBUG("set fmpsetup = 0x%04" PRIx32 "", (delay << 4) | (delay << 8));

	/*
	 * FMPVEVACCESS, based on delay.
	 */
	k = delay | (delay << 8);
	target_write_u32(target, 0xFFE8A05C, k);
	LOG_DEBUG("set fmpvevaccess = 0x%04" PRIx32 "", k);

	/*
	 * FMPCHOLD, FMPVEVHOLD, FMPVEVSETUP, based on delay.
	 */
	k <<= 1;
	target_write_u32(target, 0xFFE8A034, k);
	LOG_DEBUG("set fmpchold = 0x%04" PRIx32 "", k);
	target_write_u32(target, 0xFFE8A040, k);
	LOG_DEBUG("set fmpvevhold = 0x%04" PRIx32 "", k);
	target_write_u32(target, 0xFFE8A024, k);
	LOG_DEBUG("set fmpvevsetup = 0x%04" PRIx32 "", k);

	/*
	 * FMCVACCESS, based on delay.
	 */
	k = delay * 16;
	target_write_u32(target, 0xFFE8A060, k);
	LOG_DEBUG("set fmcvaccess = 0x%04" PRIx32 "", k);

	/*
	 * FMCSETUP, based on delay.
	 */
	k = 0x3000 | delay * 20;
	target_write_u32(target, 0xFFE8A020, k);
	LOG_DEBUG("set fmcsetup = 0x%04" PRIx32 "", k);

	/*
	 * FMEHOLD, based on delay.
	 */
	k = (delay * 20) << 2;
	target_write_u32(target, 0xFFE8A038, k);
	LOG_DEBUG("set fmehold = 0x%04" PRIx32 "", k);

	/*
	 * PWIDTH, CWIDTH, EWIDTH, based on delay.
	 */
	target_write_u32(target, 0xFFE8A050, delay * 8);
	LOG_DEBUG("set fmpwidth = 0x%04" PRIx32 "", delay * 8);
	target_write_u32(target, 0xFFE8A058, delay * 1000);
	LOG_DEBUG("set fmcwidth = 0x%04" PRIx32 "", delay * 1000);
	target_write_u32(target, 0xFFE8A054, delay * 5400);
	LOG_DEBUG("set fmewidth = 0x%04" PRIx32 "", delay * 5400);

	return result;
}

/* ---------------------------------------------------------------------- */

static int tms470_flash_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int result = ERROR_OK;
	uint32_t fmmstat;

	target_read_u32(target, 0xFFE8BC0C, &fmmstat);
	LOG_DEBUG("set fmmstat = 0x%04" PRIx32 "", fmmstat);

	if (fmmstat & 0x0080) {
		LOG_WARNING("tms470 flash command: erase still active after busy clear.");
		result = ERROR_FLASH_OPERATION_FAILED;
	}

	if (fmmstat & 0x0040) {
		LOG_WARNING("tms470 flash command: program still active after busy clear.");
		result = ERROR_FLASH_OPERATION_FAILED;
	}

	if (fmmstat & 0x0020) {
		LOG_WARNING("tms470 flash command: invalid data command.");
		result = ERROR_FLASH_OPERATION_FAILED;
	}

	if (fmmstat & 0x0010) {
		LOG_WARNING("tms470 flash command: program, erase or validate sector failed.");
		result = ERROR_FLASH_OPERATION_FAILED;
	}

	if (fmmstat & 0x0008) {
		LOG_WARNING("tms470 flash command: voltage instability detected.");
		result = ERROR_FLASH_OPERATION_FAILED;
	}

	if (fmmstat & 0x0006) {
		LOG_WARNING("tms470 flash command: command suspend detected.");
		result = ERROR_FLASH_OPERATION_FAILED;
	}

	if (fmmstat & 0x0001) {
		LOG_WARNING("tms470 flash command: sector was locked.");
		result = ERROR_FLASH_OPERATION_FAILED;
	}

	return result;
}

/* ---------------------------------------------------------------------- */

static int tms470_erase_sector(struct flash_bank *bank, int sector)
{
	uint32_t glbctrl, orig_fmregopt, fmbsea, fmbseb, fmmstat;
	struct target *target = bank->target;
	uint32_t flash_addr = bank->base + bank->sectors[sector].offset;
	int result = ERROR_OK;

	/*
	 * Set the bit GLBCTRL4 of the GLBCTRL register (in the System
	 * module) to enable writing to the flash registers }.
	 */
	target_read_u32(target, 0xFFFFFFDC, &glbctrl);
	target_write_u32(target, 0xFFFFFFDC, glbctrl | 0x10);
	LOG_DEBUG("set glbctrl = 0x%08" PRIx32 "", glbctrl | 0x10);

	/* Force normal read mode. */
	target_read_u32(target, 0xFFE89C00, &orig_fmregopt);
	target_write_u32(target, 0xFFE89C00, 0);
	LOG_DEBUG("set fmregopt = 0x%08x", 0);

	(void)tms470_flash_initialize_internal_state_machine(bank);

	/*
	 * Select one or more bits in FMBSEA or FMBSEB to disable Level 1
	 * protection for the particular sector to be erased/written.
	 */
	assert(sector >= 0);
	if (sector < 16) {
		target_read_u32(target, 0xFFE88008, &fmbsea);
		target_write_u32(target, 0xFFE88008, fmbsea | (1 << sector));
		LOG_DEBUG("set fmbsea = 0x%04" PRIx32 "", fmbsea | (1 << sector));
	} else {
		target_read_u32(target, 0xFFE8800C, &fmbseb);
		target_write_u32(target, 0xFFE8800C, fmbseb | (1 << (sector - 16)));
		LOG_DEBUG("set fmbseb = 0x%04" PRIx32 "", fmbseb | (1 << (sector - 16)));
	}
	bank->sectors[sector].is_protected = 0;

	/*
	 * clear status register, sent erase command, kickoff erase
	 */
	target_write_u16(target, flash_addr, 0x0040);
	LOG_DEBUG("write *(uint16_t *)0x%08" PRIx32 "=0x0040", flash_addr);
	target_write_u16(target, flash_addr, 0x0020);
	LOG_DEBUG("write *(uint16_t *)0x%08" PRIx32 "=0x0020", flash_addr);
	target_write_u16(target, flash_addr, 0xffff);
	LOG_DEBUG("write *(uint16_t *)0x%08" PRIx32 "=0xffff", flash_addr);

	/*
	 * Monitor FMMSTAT, busy until clear, then check and other flags for
	 * ultimate result of the operation.
	 */
	do {
		target_read_u32(target, 0xFFE8BC0C, &fmmstat);
		if (fmmstat & 0x0100)
			alive_sleep(1);
	} while (fmmstat & 0x0100);

	result = tms470_flash_status(bank);

	if (sector < 16) {
		target_write_u32(target, 0xFFE88008, fmbsea);
		LOG_DEBUG("set fmbsea = 0x%04" PRIx32 "", fmbsea);
		bank->sectors[sector].is_protected = fmbsea & (1 << sector) ? 0 : 1;
	} else {
		target_write_u32(target, 0xFFE8800C, fmbseb);
		LOG_DEBUG("set fmbseb = 0x%04" PRIx32 "", fmbseb);
		bank->sectors[sector].is_protected = fmbseb & (1 << (sector - 16)) ? 0 : 1;
	}
	target_write_u32(target, 0xFFE89C00, orig_fmregopt);
	LOG_DEBUG("set fmregopt = 0x%08" PRIx32 "", orig_fmregopt);
	target_write_u32(target, 0xFFFFFFDC, glbctrl);
	LOG_DEBUG("set glbctrl = 0x%08" PRIx32 "", glbctrl);

	return result;
}

/*----------------------------------------------------------------------
 *              Implementation of Flash Driver Interfaces
 *---------------------------------------------------------------------- */

static const struct command_registration tms470_any_command_handlers[] = {
	{
		.name = "flash_keyset",
		.usage = "<key0> <key1> <key2> <key3>",
		.handler = tms470_handle_flash_keyset_command,
		.mode = COMMAND_ANY,
		.help = "tms470 flash_keyset <key0> <key1> <key2> <key3>",
	},
	{
		.name = "osc_megahertz",
		.usage = "<MHz>",
		.handler = tms470_handle_osc_megahertz_command,
		.mode = COMMAND_ANY,
		.help = "tms470 osc_megahertz <MHz>",
	},
	{
		.name = "plldis",
		.usage = "<0 | 1>",
		.handler = tms470_handle_plldis_command,
		.mode = COMMAND_ANY,
		.help = "tms470 plldis <0/1>",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration tms470_command_handlers[] = {
	{
		.name = "tms470",
		.mode = COMMAND_ANY,
		.help = "TI tms470 flash command group",
		.usage = "",
		.chain = tms470_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/* ---------------------------------------------------------------------- */

static int tms470_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct tms470_flash_bank *tms470_info = bank->driver_priv;
	int result = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	rm57l8xx_read_part_info(bank);

	if ((first >= bank->num_sectors) || (last >= bank->num_sectors) ||
			(first > last)) {
		LOG_ERROR("Sector range %u to %u invalid.", first, last);
		return ERROR_FLASH_SECTOR_INVALID;
	}

	result = tms470_unlock_flash(bank);
	if (result != ERROR_OK)
		return result;

	for (unsigned int sector = first; sector <= last; sector++) {
		LOG_INFO("Erasing tms470 bank %u sector %u...", tms470_info->ordinal, sector);

		result = tms470_erase_sector(bank, sector);

		if (result != ERROR_OK) {
			LOG_ERROR("tms470 could not erase flash sector.");
			break;
		} else
			LOG_INFO("sector erased successfully.");
	}

	return result;
}

/* ---------------------------------------------------------------------- */

static int tms470_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct tms470_flash_bank *tms470_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t fmmac2, fmbsea, fmbseb;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	rm57l8xx_read_part_info(bank);

	if ((first >= bank->num_sectors) || (last >= bank->num_sectors) ||
			(first > last)) {
		LOG_ERROR("Sector range %u to %u invalid.", first, last);
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* enable the appropriate bank */
	target_read_u32(target, 0xFFE8BC04, &fmmac2);
	target_write_u32(target, 0xFFE8BC04, (fmmac2 & ~7) | tms470_info->ordinal);

	/* get the original sector protection flags for this bank */
	target_read_u32(target, 0xFFE88008, &fmbsea);
	target_read_u32(target, 0xFFE8800C, &fmbseb);

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		if (sector < 16) {
			fmbsea = set ? fmbsea & ~(1 << sector) : fmbsea | (1 << sector);
			bank->sectors[sector].is_protected = set ? 1 : 0;
		} else {
			fmbseb = set ? fmbseb &
				~(1 << (sector - 16)) : fmbseb | (1 << (sector - 16));
			bank->sectors[sector].is_protected = set ? 1 : 0;
		}
	}

	/* update the protection bits */
	target_write_u32(target, 0xFFE88008, fmbsea);
	target_write_u32(target, 0xFFE8800C, fmbseb);

	return ERROR_OK;
}

/* ---------------------------------------------------------------------- */

static int tms470_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t glbctrl, fmbac2, orig_fmregopt, fmbsea, fmbseb, fmmaxpp, fmmstat;
	int result = ERROR_OK;
	uint32_t i;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	rm57l8xx_read_part_info(bank);

	LOG_INFO("Writing %" PRIu32 " bytes starting at " TARGET_ADDR_FMT,
			count, bank->base + offset);

	/* set GLBCTRL.4  */
	target_read_u32(target, 0xFFFFFFDC, &glbctrl);
	target_write_u32(target, 0xFFFFFFDC, glbctrl | 0x10);

	(void)tms470_flash_initialize_internal_state_machine(bank);

	/* force max wait states */
	target_read_u32(target, 0xFFE88004, &fmbac2);
	target_write_u32(target, 0xFFE88004, fmbac2 | 0xff);

	/* save current access mode, force normal read mode */
	target_read_u32(target, 0xFFE89C00, &orig_fmregopt);
	target_write_u32(target, 0xFFE89C00, 0x00);

	/*
	 * Disable Level 1 protection for all sectors to be erased/written.
	 */
	target_read_u32(target, 0xFFE88008, &fmbsea);
	target_write_u32(target, 0xFFE88008, 0xffff);
	target_read_u32(target, 0xFFE8800C, &fmbseb);
	target_write_u32(target, 0xFFE8800C, 0xffff);

	/* read MAXPP */
	target_read_u32(target, 0xFFE8A07C, &fmmaxpp);

	for (i = 0; i < count; i += 2) {
		uint32_t addr = bank->base + offset + i;
		uint16_t word = (((uint16_t) buffer[i]) << 8) | (uint16_t) buffer[i + 1];

		if (word != 0xffff) {
			LOG_INFO("writing 0x%04x at 0x%08" PRIx32 "", word, addr);

			/* clear status register */
			target_write_u16(target, addr, 0x0040);
			/* program flash command */
			target_write_u16(target, addr, 0x0010);
			/* burn the 16-bit word (big-endian) */
			target_write_u16(target, addr, word);

			/*
			 * Monitor FMMSTAT, busy until clear, then check and other flags
			 * for ultimate result of the operation.
			 */
			do {
				target_read_u32(target, 0xFFE8BC0C, &fmmstat);
				if (fmmstat & 0x0100)
					alive_sleep(1);
			} while (fmmstat & 0x0100);

			if (fmmstat & 0x3ff) {
				LOG_ERROR("fmstat = 0x%04" PRIx32 "", fmmstat);
				LOG_ERROR(
					"Could not program word 0x%04x at address 0x%08" PRIx32 ".",
					word,
					addr);
				result = ERROR_FLASH_OPERATION_FAILED;
				break;
			}
		} else
			LOG_INFO("skipping 0xffff at 0x%08" PRIx32 "", addr);
	}

	/* restore */
	target_write_u32(target, 0xFFE88008, fmbsea);
	target_write_u32(target, 0xFFE8800C, fmbseb);
	target_write_u32(target, 0xFFE88004, fmbac2);
	target_write_u32(target, 0xFFE89C00, orig_fmregopt);
	target_write_u32(target, 0xFFFFFFDC, glbctrl);

	return result;
}

/* ---------------------------------------------------------------------- */

static int tms470_probe(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	return rm57l8xx_read_part_info(bank);
}

static int tms470_auto_probe(struct flash_bank *bank)
{
	struct tms470_flash_bank *tms470_info = bank->driver_priv;

	if (tms470_info->device_ident_reg)
		return ERROR_OK;
	return tms470_probe(bank);
}

/* ---------------------------------------------------------------------- */

static int tms470_erase_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct tms470_flash_bank *tms470_info = bank->driver_priv;
	int result = ERROR_OK;
	uint32_t fmmac2, fmbac2, glbctrl, orig_fmregopt;
	static uint8_t buffer[64 * 1024];

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!tms470_info->device_ident_reg)
		rm57l8xx_read_part_info(bank);

	/* set GLBCTRL.4  */
	target_read_u32(target, 0xFFFFFFDC, &glbctrl);
	target_write_u32(target, 0xFFFFFFDC, glbctrl | 0x10);

	/* save current access mode, force normal read mode */
	target_read_u32(target, 0xFFE89C00, &orig_fmregopt);
	target_write_u32(target, 0xFFE89C00, 0x00);

	/* enable the appropriate bank */
	target_read_u32(target, 0xFFE8BC04, &fmmac2);
	target_write_u32(target, 0xFFE8BC04, (fmmac2 & ~7) | tms470_info->ordinal);

	/* TCR = 0 */
	target_write_u32(target, 0xFFE8BC10, 0x2fc0);

	/* clear TEZ in fmbrdy */
	target_write_u32(target, 0xFFE88010, 0x0b);

	/* save current wait states, force max */
	target_read_u32(target, 0xFFE88004, &fmbac2);
	target_write_u32(target, 0xFFE88004, fmbac2 | 0xff);

	/*
	 * The TI primitives inspect the flash memory by reading one 32-bit
	 * word at a time.  Here we read an entire sector and inspect it in
	 * an attempt to reduce the JTAG overhead.
	 */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		uint32_t i, addr = bank->base + bank->sectors[sector].offset;

		LOG_INFO("checking flash bank %u sector %u", tms470_info->ordinal, sector);

		target_read_buffer(target, addr, bank->sectors[sector].size, buffer);

		bank->sectors[sector].is_erased = 1;
		for (i = 0; i < bank->sectors[sector].size; i++) {
			if (buffer[i] != 0xff) {
				bank->sectors[sector].is_erased = 0;
				break;
			}
		}
		if (bank->sectors[sector].is_erased != 1) {
			result = ERROR_FLASH_SECTOR_NOT_ERASED;
			break;
		} else
			LOG_INFO("sector erased");
	}

	/* reset TEZ, wait states, read mode, GLBCTRL.4 */
	target_write_u32(target, 0xFFE88010, 0x0f);
	target_write_u32(target, 0xFFE88004, fmbac2);
	target_write_u32(target, 0xFFE89C00, orig_fmregopt);
	target_write_u32(target, 0xFFFFFFDC, glbctrl);

	return result;
}

/* ---------------------------------------------------------------------- */

static int tms470_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct tms470_flash_bank *tms470_info = bank->driver_priv;
	int result = ERROR_OK;
	uint32_t fmmac2, fmbsea, fmbseb;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!tms470_info->device_ident_reg)
		rm57l8xx_read_part_info(bank);

	/* enable the appropriate bank */
	target_read_u32(target, 0xFFE8BC04, &fmmac2);
	target_write_u32(target, 0xFFE8BC04, (fmmac2 & ~7) | tms470_info->ordinal);

	target_read_u32(target, 0xFFE88008, &fmbsea);
	target_read_u32(target, 0xFFE8800C, &fmbseb);

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		int protected;

		if (sector < 16) {
			protected = fmbsea & (1 << sector) ? 0 : 1;
			bank->sectors[sector].is_protected = protected;
		} else {
			protected = fmbseb & (1 << (sector - 16)) ? 0 : 1;
			bank->sectors[sector].is_protected = protected;
		}

		LOG_DEBUG("bank %u sector %u is %s",
			tms470_info->ordinal,
			sector,
			protected ? "protected" : "not protected");
	}

	return result;
}

/* ---------------------------------------------------------------------- */

static int get_tms470_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct tms470_flash_bank *tms470_info = bank->driver_priv;

	if (!tms470_info->device_ident_reg)
		rm57l8xx_read_part_info(bank);

	if (!tms470_info->device_ident_reg) {
		command_print_sameline(cmd, "Cannot identify target as a TMS470\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	command_print_sameline(cmd, "\ntms470 information: Chip is %s\n", tms470_info->part_name);
	command_print_sameline(cmd, "Flash protection level 2 is %s\n",
		tms470_check_flash_unlocked(bank->target) == ERROR_OK ? "disabled" : "enabled");

	return ERROR_OK;
}

/* ---------------------------------------------------------------------- */

/*
 * flash bank tms470 <base> <size> <chip_width> <bus_width> <target>
 * [options...]
 */

FLASH_BANK_COMMAND_HANDLER(tms470_flash_bank_command)
{
	bank->driver_priv = malloc(sizeof(struct tms470_flash_bank));

	if (!bank->driver_priv)
		return ERROR_FLASH_OPERATION_FAILED;

	(void)memset(bank->driver_priv, 0, sizeof(struct tms470_flash_bank));

	return ERROR_OK;
}

const struct flash_driver rm57l8xx_flash = {
	.name = "rm57l8xx",
	.commands = tms470_command_handlers,
	.flash_bank_command = tms470_flash_bank_command,
	.erase = tms470_erase,
	.protect = tms470_protect,
	.write = tms470_write,
	.read = default_flash_read,
	.probe = tms470_probe,
	.auto_probe = tms470_auto_probe,
	.erase_check = tms470_erase_check,
	.protect_check = tms470_protect_check,
	.info = get_tms470_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
