/* 
 * File:   mdio.h
 * Author: Dannis Chen
 *
 * Created on March 1, 2025, 8:32 PM
 */

#ifndef MDIO_H
#define	MDIO_H


//#include <stdint.h>
#include <avr/io.h>

//#include "global.h"
//#include "mcc_generated_files/system/port.h"
    
#define MDIO_TIMEOUT     1400    //while loop times

#define SMI_OK  1
#define SMI_NOT_OK  0


/** Clause 22
 *  ST  OP      PHYADR  REGADR          TA      DATA
 *  ##  ##      #####   #####           ##      #### #### #### ####
 *  01
 *      10| Read
 *      01| Write
 *
 *
 *
 *  Clause 45
 *
 *  ST  OP      PHYADR  DEVTYPE         TA      ADDRESS/DATA
 *  ##  ##      #####   #####           ##      #### #### #### ####
 *  00
 *          AccessType          Device            Access Type | Contents
 *      00| Address      00000| Reserved              Address | Address
 *      01| Write        00001| PMD/PMA               Write   | Write Data
 *      11| Read         00010| WIS                   Read    | Read Data
 *      10| Post Read    00011| PCS                   Read inc| Read Data
 *          inc Addr     00100| PHY XS
 *                       00101| DTE XS
 */

typedef struct Mdio{
	uint8_t phy_addr;
	
	PORT_t* mdc_port;
	uint8_t mdc_pin_mask;
	
	PORT_t*  mdio_port;
	uint8_t mdio_pin_mask;

}Mdio;



void set_mdio_dir(Mdio *mdio);



//clause 22
void cl22_mdio_write(Mdio *mdio, uint16_t regAddr, uint16_t data);
uint16_t cl22_mdio_read(Mdio *mdio, uint16_t regAddr);

//clause 45
void cl45_mdio_write(Mdio *mdio, uint8_t device, uint16_t regAddr, uint16_t data);
uint16_t cl45_mdio_read(Mdio *mdio, uint8_t device, uint16_t regAddr);



/**
 * @brief set_mdio_reg get the register and then set one or some bits of it
 * @param mdio,
 * @param addr_dev
 * @param reg_num
 * @param val_mask  the bit(s) which will be keep unchanged, not to be set, is marked as 1.
 *                  this val_mask will execute AND operation with the original register value, and then OR operation with the value_in.
 *                  for example, if you want to set the 4th and 5th MSB bit, then the other bits should kept unchanged, so the val_mask is 0b1110 0111 1111 1111=0xe7ff.
 *
 * @param val_in    used in or operation , as described in @param val_mask
 * @return          -1 on failure 0 on success
 */
void set_mdio_reg(Mdio *mdio,   uint8_t  addr_dev, uint16_t reg_num, uint16_t val_mask, uint16_t val_in);


/**
 * @brief set_mdio_reg_whole set the whole register with an unsigned short value.
 * @param mdio
 * @param addr_dev
 * @param reg_num
 * @param val_in        the 16-bit value which will be assigned to the register.
 * @return              -1 on failure 0 on success
 */
void set_mdio_reg_whole(Mdio *mdio,   uint8_t addr_dev, uint16_t reg_num, uint16_t val_in);


/**
 * @brief get_mdio_reg   get the specific bit or bits in the register, the bits are not shifted, still stay where they are.
 * @param mdio,
 * @param addr_dev
 * @param reg_num
 * @param val_mask      the bit(s) which is(are) useful, is set 1.
 *                      for example, if you want to get the 4th MSB bit, the val_mask is 0b0001 0000 0000 0000=0x1000.
 *                      if you want to get the complete value of the register, val_mask should be 0xffff.
 * @return              the 16-bit register value
 */
uint16_t get_mdio_reg(Mdio *mdio,   uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask);



uint16_t get_mdio_reg_whole(Mdio *mdio, uint8_t addr_dev, uint16_t reg_num);

    
    
    
 void phy_addr_cl45_mdio_frame_addr(Mdio *mdio, uint8_t phy_addr,uint8_t device, uint16_t regAddr);   



void set_mdio_reg_whole_PY(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_in,uint8_t phy_addr  );
 


void set_mdio_reg_PY(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask, uint16_t val_in,uint8_t phy_addr);
 


uint16_t get_mdio_reg_PY(Mdio *mdio, uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask,uint8_t phy_addr);
 
uint16_t get_mdio_reg_whole_PY(Mdio *mdio, uint8_t addr_dev, uint16_t reg_num,uint8_t phy_addr);



#endif	/* MDIO_H */

