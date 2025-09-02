#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Marvell 10G 88x3310 PHY driver
 *
 * Based upon the ID registers, this PHY appears to be a mixture of IPs
 * from two different companies.
 *
 * There appears to be several different data paths through the PHY which
 * are automatically managed by the PHY.  The following has been determined
 * via observation and experimentation:
 *
 *       SGMII PHYXS -- BASE-T PCS -- 10G PMA -- AN -- Copper (for <= 1G)
 *  10GBASE-KR PHYXS -- BASE-T PCS -- 10G PMA -- AN -- Copper (for 10G)
 *  10GBASE-KR PHYXS -- BASE-R PCS -- Fiber
 *
 * If both the fiber and copper ports are connected, the first to gain
 * link takes priority and the other port is completely locked out.
 */
#include <linux/phy.h>
#include <linux/marvell_phy.h>

#if defined(MY_DEF_HERE)
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/netdevice.h>

extern bool convert_link_mode_to_legacy_u32(u32 *legacy_u32,
						const unsigned long *src);
#endif /* MY_DEF_HERE */

enum {
	MV_PCS_BASE_T		= 0x0000,
	MV_PCS_BASE_R		= 0x1000,
	MV_PCS_1000BASEX	= 0x2000,

	/* These registers appear at 0x800X and 0xa00X - the 0xa00X control
	 * registers appear to set themselves to the 0x800X when AN is
	 * restarted, but status registers appear readable from either.
	 */
	MV_AN_CTRL1000		= 0x8000, /* 1000base-T control register */
	MV_AN_STAT1000		= 0x8001, /* 1000base-T status register */

	/* This register appears to reflect the copper status */
	MV_AN_RESULT		= 0xa016,
	MV_AN_RESULT_SPD_10	= BIT(12),
	MV_AN_RESULT_SPD_100	= BIT(13),
	MV_AN_RESULT_SPD_1000	= BIT(14),
	MV_AN_RESULT_SPD_10000	= BIT(15),

#if defined(MY_DEF_HERE)
	/* Vendor2 MMD registers */
	MV_AN_CTRL1_MG		= 0x0020,
	MV_V2_MODE_CFG          = 0xf000,
	MV_V2_PORT_CTRL         = 0xf001,
	MV_V2_LED0_CTRL         = 0xf020,
	MV_V2_LED1_CTRL         = 0xf021,
	MV_V2_LED2_CTRL         = 0xf022,
	MV_V2_LED3_CTRL         = 0xf023,
	MV_V2_PORT_INT_MASK	= 0xf043,
	MV_V2_HOST_KR_ENABLE    = 0xf084,
	MV_V2_MAC_ADDR_LSB	= 0xf06b,
	MV_V2_MAC_ADDR_ISB	= 0xf06c,
	MV_V2_MAC_ADDR_HSB	= 0xf06d,
	MV_V2_WOL_CTRL		= 0xf06e,
	MV_V2_HOST_KR_TUNE      = 0xf07c,
	MV_V2_TEMP_CTRL         = 0xf08a,
	MV_V2_TEMP_CTRL_MASK    = 0xc000,
	MV_V2_TEMP_CTRL_SAMPLE  = 0x4000,
	MV_V2_TEMP_CTRL_DISABLE = 0xc000,
};

struct mv3310_data {
	struct phy_device *phydev;
	char *hwmon_name;
	void (*temp_read)(struct phy_device *phydev, u32 *val);
};
#endif /* MY_DEF_HERE */

static int mv3310_modify(struct phy_device *phydev, int devad, u16 reg,
			 u16 mask, u16 bits)
{
	int old, val, ret;

	old = phy_read_mmd(phydev, devad, reg);
	if (old < 0)
		return old;

	val = (old & ~mask) | (bits & mask);
	if (val == old)
		return 0;

	ret = phy_write_mmd(phydev, devad, reg, val);

	return ret < 0 ? ret : 1;
}

#if defined(MY_DEF_HERE)
/* Some PHYs within the Alaska family like 88x3310 has problems with the
 * KR Auto-negotiation. marvell datasheet for 88x3310 section 6.2.11 says that
 * KR auto-negotitaion can be enabled to adapt to the incoming SERDES by writing
 * to autoneg registers and the PMA/PMD registers
 */
static int mv3310_amd_quirk(struct phy_device *phydev)
{
	int reg=0, count=0;
	int version, subversion;

	version = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, 0xC011);
	subversion = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, 0xC012);
	dev_dbg(&phydev->mdio.dev,"%s: Marvell FW Version: %x.%x \n", __func__, version, subversion);

	reg = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_V2_HOST_KR_ENABLE);
	reg |= 0x8000;
	phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_V2_HOST_KR_ENABLE, reg);

	reg = phy_read_mmd(phydev, MDIO_MMD_PHYXS, MV_V2_HOST_KR_TUNE);
	reg = (reg & ~0x8000) | 0x4000;
	phy_write_mmd(phydev, MDIO_MMD_PHYXS, MV_V2_HOST_KR_TUNE, reg);

	if((reg & BIT(8)) && (reg & BIT(11))) {
		reg = phy_read_mmd(phydev, MDIO_MMD_AN, MV_PCS_BASE_R);

		/* disable BASE-R */
		phy_write_mmd(phydev, MDIO_MMD_AN, MV_PCS_BASE_R, reg);
	} else {
		reg = phy_read_mmd(phydev, MDIO_MMD_AN, MV_PCS_BASE_R);
		/* enable BASE-R for KR initiation */
		reg |= 0x1000;
		phy_write_mmd(phydev, MDIO_MMD_AN, MV_PCS_BASE_R, reg);
	}

	/* down the port if no link */
	reg = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_MODE_CFG);
	reg &= 0xFFF7;
	phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_MODE_CFG, reg);

	/* Do not advertise 2.5Gbe & 5GbE */
	reg = phy_read_mmd(phydev, MDIO_MMD_AN, MV_AN_CTRL1_MG);
	reg &= ~0x0180;
	phy_write_mmd(phydev, MDIO_MMD_AN, MV_AN_CTRL1_MG, reg);

	/* Do not advertise 100M & 10M */
	reg = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_ADVERTISE);
	reg &= ~0x01e0;
	phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_ADVERTISE, reg);

	/* reset port to effect above change */
	reg = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL);
	reg |= 0x8018;
	phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL, reg);

	/* wait till reset complete */

	count = 50;
	do {
		msleep(10);
		reg = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL);
	} while ((reg & 0x8000) && --count);

	if(reg & 0x8000){
		dev_err(&phydev->mdio.dev,"%s: Port Reset taking long time\n", __func__);
		return -ETIMEDOUT;
	}

	/* Set LED0 Activtiy Status LED */
	reg = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_LED0_CTRL);
	reg &= 0xE000;
	reg |= 0x128;
	phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_LED0_CTRL, reg);

	/* Set LED2 1GbE Link LED */
	reg = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_LED2_CTRL);
	reg &= 0xE000;
	reg |= 0x68;
	phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_LED2_CTRL, reg);

	/* Set LED3 10GbE Link LED */
	reg = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_LED3_CTRL);
	reg &= 0xE000;
	reg |= 0x58;
	phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_LED3_CTRL, reg);

	/* Set PCS, PMA/PMD to normal mode */
	reg = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1);
	reg &= ~0x0800;
	phy_write_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1, reg);

	reg = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_CTRL1);
	reg &= ~0x0800;
	phy_write_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_CTRL1, reg);
	
	/* PHY reusme */
	phydev->drv->resume(phydev);

	reg = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL);

	dev_dbg(&phydev->mdio.dev,"%s: quirk applied, 0x%x \n", __func__, reg);

	return 0;
}

static void mv3310_temp_read(struct phy_device *phydev, u32 *value)
{
	int temp;

	temp = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_TEMP_CTRL);
	*value = ((temp & 0xff) - 75) * 1000;
}


static int mv3310_hwmon_config(struct phy_device *phydev, bool enable)
{
       u16 val;
       int ret;

       val = enable ? MV_V2_TEMP_CTRL_SAMPLE : MV_V2_TEMP_CTRL_DISABLE;
       ret = mv3310_modify(phydev, MDIO_MMD_VEND2, MV_V2_TEMP_CTRL,
                           MV_V2_TEMP_CTRL_MASK, val);

       return ret < 0 ? ret : 0;
}

static ssize_t temp1_input_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct mv3310_data *data = dev_get_drvdata(dev);
        u32 regval;

        data->temp_read(data->phydev, &regval);
        return sprintf(buf, "%u\n", regval);
}

static DEVICE_ATTR_RO(temp1_input);

static umode_t mv3310_is_visible(struct kobject *kobj,
                                  struct attribute *attr, int index)
{
	return 0444;
}

static struct attribute *mv3310_attrs[] = {
        &dev_attr_temp1_input.attr,
        NULL
};

static const struct attribute_group mv3310_group = {
        .attrs = mv3310_attrs,
        .is_visible = mv3310_is_visible,
};
__ATTRIBUTE_GROUPS(mv3310);

static int mv3310_hwmon_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct mv3310_data *data = dev_get_drvdata(&phydev->mdio.dev);
	int ret;
	struct device *hwmon_dev;

	data->hwmon_name = devm_kstrdup(dev, dev_name(dev), GFP_KERNEL);
	if (!data->hwmon_name)
		return -ENODEV;

	ret = mv3310_hwmon_config(phydev, true);
	if (ret)
		return ret;

	data->temp_read = mv3310_temp_read;
	data->phydev = phydev;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
			data->hwmon_name, data, mv3310_groups);

	return 0;
}
#endif /* MY_DEF_HERE */


static int mv3310_probe(struct phy_device *phydev)
{
	u32 mmd_mask = MDIO_DEVS_PMAPMD | MDIO_DEVS_AN;
#if defined(MY_DEF_HERE)
	struct mv3310_data *data;
	int ret;
#endif /* MY_DEF_HERE */

	if (!phydev->is_c45 ||
	    (phydev->c45_ids.devices_in_package & mmd_mask) != mmd_mask)
		return -ENODEV;

#if defined(MY_DEF_HERE)
	data = devm_kzalloc(&phydev->mdio.dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(&phydev->mdio.dev, data);

	ret = mv3310_hwmon_probe(phydev);
	if (ret)
		return ret;
#endif /* MY_DEF_HERE */

	return 0;
}

/*
 * Resetting the MV88X3310 causes it to become non-responsive.  Avoid
 * setting the reset bit(s).
 */
static int mv3310_soft_reset(struct phy_device *phydev)
{
	return 0;
}

static int mv3310_config_init(struct phy_device *phydev)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(supported) = { 0, };
	u32 mask;
	int val;

	/* Check that the PHY interface type is compatible */
	if (phydev->interface != PHY_INTERFACE_MODE_SGMII &&
	    phydev->interface != PHY_INTERFACE_MODE_XGMII &&
	    phydev->interface != PHY_INTERFACE_MODE_XAUI &&
	    phydev->interface != PHY_INTERFACE_MODE_RXAUI &&
	    phydev->interface != PHY_INTERFACE_MODE_10GKR)
		return -ENODEV;

	__set_bit(ETHTOOL_LINK_MODE_Pause_BIT, supported);
	__set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, supported);

	if (phydev->c45_ids.devices_in_package & MDIO_DEVS_AN) {
		val = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
		if (val < 0)
			return val;

		if (val & MDIO_AN_STAT1_ABLE)
			__set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, supported);
	}

#if defined(MY_DEF_HERE)
	mv3310_amd_quirk(phydev);
#endif /* MY_DEF_HERE */
	val = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_STAT2);
	if (val < 0)
		return val;

	/* Ethtool does not support the WAN mode bits */
	if (val & (MDIO_PMA_STAT2_10GBSR | MDIO_PMA_STAT2_10GBLR |
		   MDIO_PMA_STAT2_10GBER | MDIO_PMA_STAT2_10GBLX4 |
		   MDIO_PMA_STAT2_10GBSW | MDIO_PMA_STAT2_10GBLW |
		   MDIO_PMA_STAT2_10GBEW))
		__set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, supported);
	if (val & MDIO_PMA_STAT2_10GBSR)
		__set_bit(ETHTOOL_LINK_MODE_10000baseSR_Full_BIT, supported);
	if (val & MDIO_PMA_STAT2_10GBLR)
		__set_bit(ETHTOOL_LINK_MODE_10000baseLR_Full_BIT, supported);
	if (val & MDIO_PMA_STAT2_10GBER)
		__set_bit(ETHTOOL_LINK_MODE_10000baseER_Full_BIT, supported);

	if (val & MDIO_PMA_STAT2_EXTABLE) {
		val = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_PMA_EXTABLE);
		if (val < 0)
			return val;

		if (val & (MDIO_PMA_EXTABLE_10GBT | MDIO_PMA_EXTABLE_1000BT |
			   MDIO_PMA_EXTABLE_100BTX | MDIO_PMA_EXTABLE_10BT))
			__set_bit(ETHTOOL_LINK_MODE_TP_BIT, supported);
		if (val & MDIO_PMA_EXTABLE_10GBLRM)
			__set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, supported);
		if (val & (MDIO_PMA_EXTABLE_10GBKX4 | MDIO_PMA_EXTABLE_10GBKR |
			   MDIO_PMA_EXTABLE_1000BKX))
			__set_bit(ETHTOOL_LINK_MODE_Backplane_BIT, supported);
		if (val & MDIO_PMA_EXTABLE_10GBLRM)
			__set_bit(ETHTOOL_LINK_MODE_10000baseLRM_Full_BIT,
				  supported);
		if (val & MDIO_PMA_EXTABLE_10GBT)
			__set_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
				  supported);
		if (val & MDIO_PMA_EXTABLE_10GBKX4)
			__set_bit(ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT,
				  supported);
		if (val & MDIO_PMA_EXTABLE_10GBKR)
			__set_bit(ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
				  supported);
		if (val & MDIO_PMA_EXTABLE_1000BT)
			__set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				  supported);
		if (val & MDIO_PMA_EXTABLE_1000BKX)
			__set_bit(ETHTOOL_LINK_MODE_1000baseKX_Full_BIT,
				  supported);
		if (val & MDIO_PMA_EXTABLE_100BTX)
			__set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
				  supported);
		if (val & MDIO_PMA_EXTABLE_10BT)
			__set_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT,
				  supported);
	}

#if defined(MY_DEF_HERE)
	if (!convert_link_mode_to_legacy_u32(&mask, supported))
#else /* MY_DEF_HERE */
	if (!ethtool_convert_link_mode_to_legacy_u32(&mask, supported))
#endif /* MY_DEF_HERE */
		dev_warn(&phydev->mdio.dev,
			 "PHY supports (%*pb) more modes than phylib supports, some modes not supported.\n",
			 __ETHTOOL_LINK_MODE_MASK_NBITS, supported);

	phydev->supported &= mask;
	phydev->advertising &= phydev->supported;

	return 0;
}

static int mv3310_config_aneg(struct phy_device *phydev)
{
	bool changed = false;
	u32 advertising;
	int ret;

	if (phydev->autoneg == AUTONEG_DISABLE) {
		ret = genphy_c45_pma_setup_forced(phydev);
		if (ret < 0)
			return ret;

		return genphy_c45_an_disable_aneg(phydev);
	}

	phydev->advertising &= phydev->supported;
	advertising = phydev->advertising;

	ret = mv3310_modify(phydev, MDIO_MMD_AN, MDIO_AN_ADVERTISE,
			    ADVERTISE_ALL | ADVERTISE_100BASE4 |
			    ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM,
			    ethtool_adv_to_mii_adv_t(advertising));
	if (ret < 0)
		return ret;
	if (ret > 0)
		changed = true;

	ret = mv3310_modify(phydev, MDIO_MMD_AN, MV_AN_CTRL1000,
			    ADVERTISE_1000FULL | ADVERTISE_1000HALF,
			    ethtool_adv_to_mii_ctrl1000_t(advertising));
	if (ret < 0)
		return ret;
	if (ret > 0)
		changed = true;

	/* 10G control register */
	ret = mv3310_modify(phydev, MDIO_MMD_AN, MDIO_AN_10GBT_CTRL,
			    MDIO_AN_10GBT_CTRL_ADV10G,
			    advertising & ADVERTISED_10000baseT_Full ?
				MDIO_AN_10GBT_CTRL_ADV10G : 0);
	if (ret < 0)
		return ret;
	if (ret > 0)
		changed = true;

	if (changed)
		ret = genphy_c45_restart_aneg(phydev);

	return ret;
}

static int mv3310_aneg_done(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_PCS_BASE_R + MDIO_STAT1);
	if (val < 0)
		return val;

	if (val & MDIO_STAT1_LSTATUS)
		return 1;

	return genphy_c45_aneg_done(phydev);
}

/* 10GBASE-ER,LR,LRM,SR do not support autonegotiation. */
static int mv3310_read_10gbr_status(struct phy_device *phydev)
{
	phydev->link = 1;
	phydev->speed = SPEED_10000;
	phydev->duplex = DUPLEX_FULL;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII)
		phydev->interface = PHY_INTERFACE_MODE_10GKR;

	return 0;
}

static int mv3310_read_status(struct phy_device *phydev)
{
	u32 mmd_mask = phydev->c45_ids.devices_in_package;
	int val;

	/* The vendor devads do not report link status.  Avoid the PHYXS
	 * instance as there are three, and its status depends on the MAC
	 * being appropriately configured for the negotiated speed.
	 */
	mmd_mask &= ~(BIT(MDIO_MMD_VEND1) | BIT(MDIO_MMD_VEND2) |
		      BIT(MDIO_MMD_PHYXS));

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->lp_advertising = 0;
	phydev->link = 0;
	phydev->pause = 0;
	phydev->asym_pause = 0;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MV_PCS_BASE_R + MDIO_STAT1);
	if (val < 0)
		return val;

	if (val & MDIO_STAT1_LSTATUS)
		return mv3310_read_10gbr_status(phydev);

	val = genphy_c45_read_link(phydev, mmd_mask);
	if (val < 0)
		return val;

	phydev->link = val > 0 ? 1 : 0;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
	if (val < 0)
		return val;

	if (val & MDIO_AN_STAT1_COMPLETE) {
		val = genphy_c45_read_lpa(phydev);
		if (val < 0)
			return val;

		/* Read the link partner's 1G advertisment */
		val = phy_read_mmd(phydev, MDIO_MMD_AN, MV_AN_STAT1000);
		if (val < 0)
			return val;

		phydev->lp_advertising |= mii_stat1000_to_ethtool_lpa_t(val);

		if (phydev->autoneg == AUTONEG_ENABLE) {
			val = phy_read_mmd(phydev, MDIO_MMD_AN, MV_AN_RESULT);
			if (val < 0)
				return val;

			if (val & MV_AN_RESULT_SPD_10000)
				phydev->speed = SPEED_10000;
			else if (val & MV_AN_RESULT_SPD_1000)
				phydev->speed = SPEED_1000;
			else if (val & MV_AN_RESULT_SPD_100)
				phydev->speed = SPEED_100;
			else if (val & MV_AN_RESULT_SPD_10)
				phydev->speed = SPEED_10;

			phydev->duplex = DUPLEX_FULL;
		}
	}

	if (phydev->autoneg != AUTONEG_ENABLE) {
		val = genphy_c45_read_pma(phydev);
		if (val < 0)
			return val;
	}

	if ((phydev->interface == PHY_INTERFACE_MODE_SGMII ||
	     phydev->interface == PHY_INTERFACE_MODE_10GKR) && phydev->link) {
		/* The PHY automatically switches its serdes interface (and
		 * active PHYXS instance) between Cisco SGMII and 10GBase-KR
		 * modes according to the speed.  Florian suggests setting
		 * phydev->interface to communicate this to the MAC. Only do
		 * this if we are already in either SGMII or 10GBase-KR mode.
		 */
		if (phydev->speed == SPEED_10000)
			phydev->interface = PHY_INTERFACE_MODE_10GKR;
		else if (phydev->speed >= SPEED_10 &&
			 phydev->speed < SPEED_10000)
			phydev->interface = PHY_INTERFACE_MODE_SGMII;
	}

	return 0;
}

#if defined(MY_DEF_HERE)
static int syno_set_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	int ret, val;

	/* Force PHY not the advertise 10GbE */
	val = phy_read_mmd(phydev, MDIO_MMD_AN, MV_AN_CTRL1_MG);
	val &= ~0x1000;
	ret = phy_write_mmd(phydev, MDIO_MMD_AN, MV_AN_CTRL1_MG, val);
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to config advertise\n", __func__);
		return ret;
	}

	/* advertise 100M & 10M */
	val = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_ADVERTISE);
	val |= 0x01e0;
	ret = phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_ADVERTISE, val);
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to config advertise\n", __func__);
		return ret;
	}

	/* Enable PHY WOL interrupt */
	val = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_INT_MASK);
	val |= 0x0100;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_INT_MASK, val);
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to enable wol int\n", __func__);
		return ret;
	}

	/* Set MAC address for magic packet detection */
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_MAC_ADDR_HSB,
			((phydev->attached_dev->dev_addr[5] << 8) |
			 phydev->attached_dev->dev_addr[4]));
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to set MAC address HSB\n", __func__);
		return ret;
	}
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_MAC_ADDR_ISB,
			((phydev->attached_dev->dev_addr[3] << 8) |
			 phydev->attached_dev->dev_addr[2]));
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to set MAC address ISB\n", __func__);
		return ret;
	}
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_MAC_ADDR_LSB,
			((phydev->attached_dev->dev_addr[1] << 8) |
			 phydev->attached_dev->dev_addr[0]));
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to set MAC address LSB\n", __func__);
		return ret;
	}

	/* Magic packet detection enabled */
	val = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_WOL_CTRL);
	val |= 0x01;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_WOL_CTRL, val);
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to enable magic packet detection \n", __func__);
		return ret;
	}

	/* Softreset to effect above change */
	val = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL);
	val |= 0x8000;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL, val);
	if (ret) {
		dev_err(&phydev->mdio.dev,"%s: failed to softreset phy \n", __func__);
		return ret;
	}
	msleep(500);

	return 0;
}

static int mv3310_suspend(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL);
	val |= 0x0800;
	return phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL, val);
}

static int mv3310_resume(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL);
	val &= ~0x0800;
	return phy_write_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL, val);
}
#endif /* MY_DEF_HERE */

static struct phy_driver mv3310_drivers[] = {
	{
		.phy_id		= 0x002b09aa,
		.phy_id_mask	= MARVELL_PHY_ID_MASK,
		.name		= "mv88x3310",
		.features	= SUPPORTED_10baseT_Full |
				  SUPPORTED_100baseT_Full |
				  SUPPORTED_1000baseT_Full |
				  SUPPORTED_Autoneg |
				  SUPPORTED_TP |
				  SUPPORTED_FIBRE |
				  SUPPORTED_10000baseT_Full |
				  SUPPORTED_Backplane,
		.probe		= mv3310_probe,
		.soft_reset	= mv3310_soft_reset,
		.config_init	= mv3310_config_init,
		.config_aneg	= mv3310_config_aneg,
		.aneg_done	= mv3310_aneg_done,
		.read_status	= mv3310_read_status,
#if defined(MY_DEF_HERE)
		.set_wol	= syno_set_wol,
		.suspend	= mv3310_suspend,
		.resume		= mv3310_resume,
#endif /* MY_DEF_HERE */
	},
};

module_phy_driver(mv3310_drivers);

static struct mdio_device_id __maybe_unused mv3310_tbl[] = {
	{ 0x002b09aa, MARVELL_PHY_ID_MASK },
	{ },
};
MODULE_DEVICE_TABLE(mdio, mv3310_tbl);
MODULE_DESCRIPTION("Marvell Alaska X 10Gigabit Ethernet PHY driver (MV88X3310)");
MODULE_LICENSE("GPL");
