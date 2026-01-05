#include "mdio.h"
//#include "stdio.h"
//#include <string.h>

//#include <var/io.h>
//#include "var/delay.h"
#include "global.h"
//#include <avr/io.h>    
#include <avr/interrupt.h>

#define MDIO_DELAY          2	// us
#define MDIO_READ_DELAY     2 	// us

/*  Or MII_ADDR_C45 into regnum for read/write on mii_bus to enable the 21 bit
 *   IEEE 802.3ae clause 45 addressing mode used by 10GIGE phy chips.
 */
#define MDIO_CL45_ADDR  	0x00

#define MDIO_CL45_WRITE 	0x01
#define MDIO_CL45_READ  	0x11
#define MDIO_CL45_READ_INC 	0x10

#define MDIO_CL22_READ  	0x2

#define MDIO_CL22_WRITE 	0x1







// 波特率参数
#define BAUD_RATE 9600
#define BIT_DELAY_US (2450000UL / BAUD_RATE) // 104μs @9600bps  83uS

 



 
#include <util/delay.h> // AVR延时函数库

// 精确微秒级延时（误差±0.5us @16MHz）
static inline void bit_delay(uint32_t maxCount) {
    _delay_us(maxCount ); // 补偿函数调用开销
}

// 精确纳秒级校准（用于高波特率）
#define NOP() __asm__ __volatile__ ("nop")
static void calibrate_delay(void) {
    for(uint8_t i=0; i<5; i++) NOP(); // 根据实际测量调整
}








inline void mdioDelay(uint32_t maxCount)
{
   volatile uint32_t count = 0;
    
 //  _delay_us(maxCount );   
      _delay_us(maxCount );   
 //  while (count <maxCount)
 count++;
 count=0;
}

/************************************************************************
 only set the direction.                                      
************************************************************************/
void set_mdio_dir(Mdio *mdio)
{
    SET_PIN_DIR_OUT(mdio->mdc_port, mdio->mdc_pin_mask);
    SET_PIN_DIR_OUT(mdio->mdio_port, mdio->mdio_pin_mask);
}



/**
 * @brief mdio_write_bit
 * output one bit to MDIO port
 */
static void mdio_write_bit(Mdio *mdio, uint8_t val)
{
   	mdioDelay(MDIO_DELAY); 
	SET_PIN_OUT_HIGH(mdio->mdc_port, mdio->mdc_pin_mask);
	
	mdioDelay(MDIO_DELAY);
	
	if(val)
		SET_PIN_OUT_HIGH(mdio->mdio_port, mdio->mdio_pin_mask);
	else
		SET_PIN_OUT_LOW(mdio->mdio_port, mdio->mdio_pin_mask);

	mdioDelay(MDIO_DELAY);
	
	SET_PIN_OUT_LOW(mdio->mdc_port, mdio->mdc_pin_mask);

}

/**
 * @brief mdio_receive_bit
 * read one bit from MDIO port
 */
static uint8_t mdio_read_bit(Mdio *mdio)
{
	uint8_t value;
		
    mdioDelay(MDIO_DELAY);
	SET_PIN_OUT_HIGH(mdio->mdc_port, mdio->mdc_pin_mask);

	mdioDelay(MDIO_READ_DELAY);

	SET_PIN_OUT_LOW(mdio->mdc_port, mdio->mdc_pin_mask);

	value = GET_PIN_IN(mdio->mdio_port, mdio->mdio_pin_mask);
	
	mdioDelay(MDIO_DELAY);

	return value;
}

/**
 * @brief mdio_write_num
 * mdio write consecutive number of bit
 * @param value
 * @param nBits
 */
static void mdio_write_bits_num(Mdio *mdio,uint32_t value ,uint32_t nBits)
{
    char tt;
    
    uint32_t i;
    
	for(  i = nBits-1 ; i >= 0; i--){
		
      mdio_write_bit(mdio,(value >> i) & 1);
    
      //if(i>50){   
      //    tt=0;
       //   break;}
	}
    
    tt=0;
}

/**
 * @brief nmdio_read_num
 * mdio read consecutive number of bit
 * @param value
 * @param nBits
 */
static int  mdio_read_bits_num(Mdio *mdio, uint32_t nBits)
{
	uint8_t i;
	uint8_t ret = 0;
	for(i = nBits ; i > 0; i--)
	{
		ret <<= 1;
		ret |= mdio_read_bit(mdio);
	}

	return ret;
}

static void cl45_mdio_frame_addr(Mdio *mdio, uint8_t device, uint16_t regAddr)
{

	SET_PIN_DIR_OUT(mdio->mdio_port, mdio->mdio_pin_mask); // MUST !!!

	/*閸欐垿锟斤拷32bit閻拷1閿涘矁绻栨稉顏勬姎閸撳秶绱戦崺鐔剁瑝閺勵垰绻�妞よ崵娈戦敍灞剧厙娴滄稓澧块悶鍡楃湴閼侯垳澧栭惃鍑狣IO閹垮秳缍旂亸杈ㄧ梾閺堝绻栨稉顏勭厵*/
	for(uint8_t i = 0; i < 32; i++)
		mdio_write_bit(mdio,1);

	/* 閸欐垿锟戒讣tart of Frame(ST): clause 45: 00 */
	mdio_write_bit(mdio,0);
	mdio_write_bit(mdio,0);

	/* Operation Code(OP): 閸︽澘娼冮幙宥勭稊MDIO_CL45_ADDR  0*/
	mdio_write_bit(mdio,(MDIO_CL45_ADDR>> 1) & 1);
	mdio_write_bit(mdio, MDIO_CL45_ADDR & 1);


	/* 閸欐垿锟戒赋HY Address(PHYAD) */
	mdio_write_bits_num(mdio,           mdio->phy_addr,5);

	/* 閸欐垿锟戒笍evice */
	mdio_write_bits_num(mdio,device,5);

	/* 閸欐垿锟戒辜urn around, write閿涳拷10 */
	mdio_write_bit(mdio,1);
	mdio_write_bit(mdio,0);

	/* 閸欐垿锟戒龚ata */
	mdio_write_bits_num(mdio,regAddr,16);

	//    mdio_set_bit(1);	//鐠佸彞璐熸姗堢礉娴ｆ粈璐熺敮褍澧犵紓锟介崺锟�

}

/**
 * write鐢冩嫲address鐢呮畱娴狅絿鐖滈弰顖欑閺嶉娈戦敍灞藉涧閺勵垱娓堕崥搴濈鐞涘苯鍟撻崗銉ф畱閸愬懎顔愭稉宥呮倱閵嗭拷
 */
static void cl45_mdio_frame_write(Mdio *mdio, uint8_t device, uint16_t data)
{
//	SET_PIN_DIR_OUT(mdio->mdio_port, mdio->mdio_pin_mask);  //no need, because cl45 need write address first. direction remain unchanged.

	/*閸欐垿锟斤拷32bit閻拷1閿涘矁绻栨稉顏勬姎閸撳秶绱戦崺鐔剁瑝閺勵垰绻�妞よ崵娈戦敍灞剧厙娴滄稓澧块悶鍡楃湴閼侯垳澧栭惃鍑狣IO閹垮秳缍旂亸杈ㄧ梾閺堝绻栨稉顏勭厵*/
	for(uint8_t i = 0; i < 32; i++)
		mdio_write_bit(mdio,1);

	/* 閸欐垿锟戒讣tart of Frame(ST): clause 45: 00 */
	mdio_write_bit(mdio,0);
	mdio_write_bit(mdio,0);

	/* Operation Code(OP): 鐠囩粯鎼锋担锟�(10)鏉╂ɑ妲搁崘娆愭惙娴ｏ拷(01)*/
	mdio_write_bit(mdio,(MDIO_CL45_WRITE>> 1) & 1);
	mdio_write_bit(mdio,MDIO_CL45_WRITE & 1);


	/* 閸欐垿锟戒赋HY Address(PHYAD) */
	mdio_write_bits_num(mdio,mdio->phy_addr,5);

	/* 閸欐垿锟戒笍evice */
	mdio_write_bits_num(mdio,device,5);

	/* 閸欐垿锟戒辜urn around, write閿涳拷10 */
	mdio_write_bit(mdio,1);
	mdio_write_bit(mdio,0);

	/* 閸欐垿锟戒龚ata */
	mdio_write_bits_num(mdio,data,16);

}

static int cl45_mdio_frame_read(Mdio *mdio, uint8_t device)
{
//	SET_PIN_DIR_OUT(mdio->mdio_port, mdio->mdio_pin_mask); //no need, because cl45 need write address first. direction remain unchanged.
cli();

	/*閸欐垿锟斤拷32bit閻拷1閿涘矁绻栨稉顏勬姎閸撳秶绱戦崺鐔剁瑝閺勵垰绻�妞よ崵娈戦敍灞剧厙娴滄稓澧块悶鍡楃湴閼侯垳澧栭惃鍑狣IO閹垮秳缍旂亸杈ㄧ梾閺堝绻栨稉顏勭厵*/
	for(uint8_t i = 0; i < 32; i++)
		mdio_write_bit(mdio,1);

	/* 閸欐垿锟戒讣tart of Frame(ST): clause 45: 00 */
	mdio_write_bit(mdio,0);
	mdio_write_bit(mdio,0);

	/* Operation Code(OP): 鐠囩粯鎼锋担锟�(11)鏉╂ɑ妲搁崘娆愭惙娴ｏ拷(01)*/
	//  mdio_write_bit((MDIO_CL45_READ>> 1) & 1);	// (MDIO_CL45_READ>> 1) & 1 閿涘矂姣﹂柆鎾剁搼娴滐拷0閿涳拷
	//  mdio_write_bit( MDIO_CL45_READ & 1);		//  MDIO_CL45_READ & 1
	mdio_write_bit(mdio,1);
	mdio_write_bit(mdio,1);

	/* 閸欐垿锟戒赋HY Address(PHYAD) */
	mdio_write_bits_num(mdio, mdio->phy_addr,5);

	/* 閸欐垿锟戒笍evice */
	mdio_write_bits_num(mdio, device,5);

	/* 閸欐垿锟戒辜urn around, write閿涳拷10 */
	mdio_write_bit(mdio,1);


	SET_PIN_DIR_IN(mdio->mdio_port, mdio->mdio_pin_mask); 


	/*  check the turnaround bit: the PHY should be driving it to zero */
	if(mdio_read_bit(mdio) != 0) //PHY鐏忓棙锟借崵鍤庣純顔荤秵.
	{
		/* PHY didn't driver TA low -- flush any bits it may be trying to send*/
		for(uint8_t i = 0; i < 32; i++)
			mdio_read_bit(mdio);
        
		printf(" C45 PHY didn't driver TA low");
        return 0xFFFF;
	}

	int data = mdio_read_bits_num(mdio,16);
sei();
	return data;



}




void phy_addr_cl45_mdio_frame_addr(Mdio *mdio, uint8_t phy_addr,uint8_t device, uint16_t regAddr)
{
cli();
	SET_PIN_DIR_OUT(mdio->mdio_port, mdio->mdio_pin_mask); // MUST !!!

	/*閸欐垿锟斤拷32bit閻拷1閿涘矁绻栨稉顏勬姎閸撳秶绱戦崺鐔剁瑝閺勵垰绻�妞よ崵娈戦敍灞剧厙娴滄稓澧块悶鍡楃湴閼侯垳澧栭惃鍑狣IO閹垮秳缍旂亸杈ㄧ梾閺堝绻栨稉顏勭厵*/
	for(uint8_t i = 0; i < 32; i++)
		mdio_write_bit(mdio,1);

	/* 閸欐垿锟戒讣tart of Frame(ST): clause 45: 00 */
	mdio_write_bit(mdio,0);
	mdio_write_bit(mdio,0);

	/* Operation Code(OP): 閸︽澘娼冮幙宥勭稊MDIO_CL45_ADDR  0*/
	mdio_write_bit(mdio,(MDIO_CL45_ADDR>> 1) & 1);
	mdio_write_bit(mdio, MDIO_CL45_ADDR & 1);


	/* 閸欐垿锟戒赋HY Address(PHYAD) */
	mdio_write_bits_num(mdio,           mdio->phy_addr,5);

	/* 閸欐垿锟戒笍evice */
	mdio_write_bits_num(mdio,device,5);

	/* 閸欐垿锟戒辜urn around, write閿涳拷10 */
	mdio_write_bit(mdio,1);
	mdio_write_bit(mdio,0);

	/* 閸欐垿锟戒龚ata */
	mdio_write_bits_num(mdio,regAddr,16);
sei();
	//    mdio_set_bit(1);	//鐠佸彞璐熸姗堢礉娴ｆ粈璐熺敮褍澧犵紓锟介崺锟�

}








void cl45_mdio_write_phy_addr(Mdio *mdio, uint8_t device, uint16_t regAddr, uint16_t data, uint8_t phy_addr)
{
  cli();
    mdio->phy_addr=phy_addr;
	cl45_mdio_frame_addr(mdio, device, regAddr);

	cl45_mdio_frame_write(mdio, device, data);
sei();
	return;
}

uint16_t cl45_mdio_read_phy_addr(Mdio *mdio, uint8_t device, uint16_t regAddr, uint8_t phy_addr)
{
    mdio->phy_addr=phy_addr;
	cl45_mdio_frame_addr(mdio, device, regAddr);

	return cl45_mdio_frame_read(mdio,  device);
}




void cl45_mdio_write(Mdio *mdio, uint8_t device, uint16_t regAddr, uint16_t data)
{  
  cli();
	cl45_mdio_frame_addr(mdio, device, regAddr);

	cl45_mdio_frame_write(mdio, device, data);
sei();
	return;
}

uint16_t cl45_mdio_read(Mdio *mdio, uint8_t device, uint16_t regAddr)
{
    
	cl45_mdio_frame_addr(mdio, device, regAddr);

	return cl45_mdio_frame_read(mdio,  device);
}





void cl22_mdio_write(Mdio *mdio, uint16_t regAddr, uint16_t data)
{
	SET_PIN_DIR_OUT(mdio->mdio_port, mdio->mdio_pin_mask); //MUST!!!

	/*閸欐垿锟斤拷32bit閻拷1閿涘矁绻栨稉顏勬姎閸撳秶绱戦崺鐔剁瑝閺勵垰绻�妞よ崵娈戦敍灞剧厙娴滄稓澧块悶鍡楃湴閼侯垳澧栭惃鍑狣IO閹垮秳缍旂亸杈ㄧ梾閺堝绻栨稉顏勭厵*/
	for(uint8_t i = 0; i < 32; i++)
		mdio_write_bit(mdio, 1);

	/* 閸欐垿锟戒讣tart of Frame(ST): clause 22: 01 */
	mdio_write_bit(mdio,0);
	mdio_write_bit(mdio,1);

	/* Operation Code(OP): CL22鐠囩粯鎼锋担锟�(01)鏉╂ɑ妲搁崘娆愭惙娴ｏ拷(10)*/
	mdio_write_bit(mdio,(MDIO_CL22_WRITE>> 1) & 1);
	mdio_write_bit(mdio, MDIO_CL22_WRITE & 1);


	/* 閸欐垿锟戒赋HY Address(PHYAD) */
	mdio_write_bits_num(mdio, mdio->phy_addr,5);

	/* 閸欐垿锟戒购egAddr */
	mdio_write_bits_num(mdio, regAddr,5);

	/* 閸欐垿锟戒辜urn around, write閿涳拷10 */
	mdio_write_bit(mdio,1);
	mdio_write_bit(mdio,0);

	/* 閸欐垿锟戒龚ata */
	mdio_write_bits_num(mdio, data,16);
}

uint16_t cl22_mdio_read(Mdio *mdio, uint16_t regAddr)
{
	SET_PIN_DIR_OUT(mdio->mdio_port, mdio->mdio_pin_mask);  //MUST

	/*閸欐垿锟斤拷32bit閻拷1閿涘矁绻栨稉顏勬姎閸撳秶绱戦崺鐔剁瑝閺勵垰绻�妞よ崵娈戦敍灞剧厙娴滄稓澧块悶鍡楃湴閼侯垳澧栭惃鍑狣IO閹垮秳缍旂亸杈ㄧ梾閺堝绻栨稉顏勭厵*/
	for(uint8_t i = 0; i < 32; i++)
		mdio_write_bit(mdio,1);

	/* 閸欐垿锟戒讣tart of Frame(ST): clause 22: 01 */
	mdio_write_bit(mdio,0);
	mdio_write_bit(mdio,1);

	/* Operation Code(OP): CL22鐠囩粯鎼锋担锟�(10)鏉╂ɑ妲搁崘娆愭惙娴ｏ拷(01)*/
	//    mdio_write_bit((MDIO_CL22_READ >> 1) & 1);
	//    mdio_write_bit(MDIO_CL22_READ & 1);
	mdio_write_bit(mdio,1);
	mdio_write_bit(mdio,0);

	/* 閸欐垿锟戒赋HY Address(PHYAD) */
	mdio_write_bits_num(mdio, mdio->phy_addr,5);

	/* 閸欐垿锟戒购egAddr */
	mdio_write_bits_num(mdio, regAddr,5);

	/* 閸欐垿锟戒辜urn around, write閿涳拷10 */
	mdio_write_bit(mdio,1);

	SET_PIN_DIR_IN(mdio->mdio_port, mdio->mdio_pin_mask);

	/*  check the turnaround bit: the PHY should be driving it to zero */
	if(mdio_read_bit(mdio) != 0) //PHY鐏忓棙锟借崵鍤庣純顔荤秵.
	{
		/* PHY didn't driver TA low -- flush any bits it may be trying to send*/
		for(uint8_t i = 0; i < 32; i++)
			mdio_read_bit(mdio);
        
        		
        printf("C22 PHY didn't driver TA low");
		return 0xFFFF;
	}

	int data = mdio_read_bits_num(mdio,16);

	return data;
}



/************************************************
 * 进一步的封装
 ************************************************/

void set_mdio_reg_whole(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_in)
{
    
	cl45_mdio_frame_addr(mdio, addr_dev, reg_num);

	cl45_mdio_frame_write(mdio, addr_dev, val_in);

}



void set_mdio_reg(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask, uint16_t val_in)
{
    uint16_t data =cl45_mdio_read(mdio, addr_dev, reg_num);

//    data  = data & val_mask;
//    data |= val_in;
    
    data  = (data & val_mask) | val_in;

    cl45_mdio_write(mdio, addr_dev, reg_num, data);

}


uint16_t get_mdio_reg(Mdio *mdio, uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask)
{
    return val_mask & cl45_mdio_read(mdio, addr_dev, reg_num);

}

uint16_t get_mdio_reg_whole(Mdio *mdio, uint8_t addr_dev, uint16_t reg_num)
{
    return cl45_mdio_read(mdio, addr_dev, reg_num);

}




/////////////////////////////////////////////////////////////////////////////////////////////



void set_mdio_reg_whole_PY(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_in,uint8_t phy_addr  )
{
        mdio->phy_addr=phy_addr;
	cl45_mdio_frame_addr(mdio, addr_dev, reg_num);

	cl45_mdio_frame_write(mdio, addr_dev, val_in);

}



void set_mdio_reg_PY(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask, uint16_t val_in,uint8_t phy_addr)
{    uint16_t data;
          
mdio->phy_addr=phy_addr;
    
data =cl45_mdio_read(mdio, addr_dev, reg_num);

//    data  = data & val_mask;
//    data |= val_in;
    
    data  = (data & val_mask) | val_in;

    cl45_mdio_write(mdio, addr_dev, reg_num, data);

}


uint16_t get_mdio_reg_PY(Mdio *mdio, uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask,uint8_t phy_addr)
{
    
    mdio->phy_addr=phy_addr;
       
    printf("pyhaddr %d",mdio->phy_addr);
    return val_mask & cl45_mdio_read(mdio, addr_dev, reg_num);

}

uint16_t get_mdio_reg_whole_PY(Mdio *mdio, uint8_t addr_dev, uint16_t reg_num,uint8_t phy_addr)
{
    mdio->phy_addr=phy_addr;
    printf("pyh45  addr %d",mdio->phy_addr);
    
    return cl45_mdio_read(mdio, addr_dev, reg_num);

}



