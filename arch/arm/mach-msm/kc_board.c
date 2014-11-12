/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/kc_board.h>

#define     OEM_PULLDOWN            0x00000001
#define     OEM_NONEPULL            0x00000000

#define     HIGH_VALUE              0x01
#define     LOW_VALUE               0x00

#define     BOARD_CHECK1_NUM        145
#define     BOARD_CHECK2_NUM        144
#define     BOARD_CHECK3_NUM         94

#define     PORT_MASK_LLL           0x00
#define     PORT_MASK_LLH           0x01
#define     PORT_MASK_LHL           0x02
#define     PORT_MASK_LHH           0x03
#define     PORT_MASK_HLL           0x04
#define     PORT_MASK_HLH           0x05
#define     PORT_MASK_HHL           0x06
#define     PORT_MASK_HHH           0x07

#define BOARD_CHECK1_NO_PULL \
   GPIO_CFG(BOARD_CHECK1_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define BOARD_CHECK2_NO_PULL \
   GPIO_CFG(BOARD_CHECK2_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define BOARD_CHECK3_NO_PULL \
   GPIO_CFG(BOARD_CHECK3_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)

static uint32_t kc_board_cfg[] = {
   GPIO_CFG(BOARD_CHECK1_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
   GPIO_CFG(BOARD_CHECK2_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
   GPIO_CFG(BOARD_CHECK3_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static oem_board_type _board_type = OEM_UNSET_TYPE;
static oem_wireless_charge_type _board_wlc_type = OEM_WLC_UNSET_TYPE;

/*===========================================================================
  FUNCTION OEM_board_judgement()

  PARAMETERS
    None.

  RETURN VALUE
    None.
===========================================================================*/
void OEM_board_judgement(void)
{
  int _value_check1,_value_check2,_value_check3 = LOW_VALUE;
  int num, result=0;
  uint8_t               _port_mask=0;
  
  result = gpio_request( BOARD_CHECK1_NUM, "board_check1" );
  
  if( result != 0 )
  {
     printk( KERN_ERR "OEM_board_judgement() : gpio_request() Failed PORT=%d",
                                                             BOARD_CHECK1_NUM );
  }
  
  result = gpio_request( BOARD_CHECK2_NUM, "board_check2" );
  
  if( result != 0 )
  {
     printk( KERN_ERR "OEM_board_judgement() : gpio_request() Failed PORT=%d",
                                                             BOARD_CHECK2_NUM );
  }
  
  result = gpio_request( BOARD_CHECK3_NUM, "board_check3" );
  
  if( result != 0 )
  {
     printk( KERN_ERR "OEM_board_judgement() : gpio_request() Failed PORT=%d",
                                                             BOARD_CHECK3_NUM );
  }
  
  for( num=0 ; num < ARRAY_SIZE(kc_board_cfg); num++ )
  {
     result = gpio_tlmm_config( kc_board_cfg[num], GPIO_CFG_ENABLE);
     
     if( result != 0 )
     {
        printk( KERN_ERR "OEM_board_judgement() : Initialized num=%d Error", num );
     }
  }
  
  _value_check1 = gpio_get_value( BOARD_CHECK1_NUM );
  _value_check2 = gpio_get_value( BOARD_CHECK2_NUM );
  _value_check3 = gpio_get_value( BOARD_CHECK3_NUM );
  
  _port_mask = ((uint8_t)_value_check3 | ((uint8_t)_value_check2 << 1 ) | ((uint8_t)_value_check1 << 2 ));
  
  switch( _port_mask ){
    case PORT_MASK_LLL:
       _board_type = OEM_BOARD0_TYPE;
       _board_wlc_type = OEM_WLC_OFF_BOARD_TYPE;

       break;
       
    case PORT_MASK_LLH:
        _board_type = OEM_BOARD1_TYPE;
       _board_wlc_type = OEM_WLC_OFF_BOARD_TYPE;

       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE );
       break;
       
    case PORT_MASK_LHL:
       _board_type = OEM_BOARD2_TYPE;
       _board_wlc_type = OEM_WLC_OFF_BOARD_TYPE;

       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       break;
       
    case PORT_MASK_LHH:
       _board_type = OEM_BOARD3_TYPE;
       _board_wlc_type = OEM_WLC_OFF_BOARD_TYPE;

       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE );
       break;
       
    case PORT_MASK_HLL:
       _board_type = OEM_BOARD0_TYPE;
       _board_wlc_type = OEM_WLC_ON_BOARD_TYPE;

       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       break;
       
    case PORT_MASK_HLH:
       _board_type = OEM_BOARD1_TYPE;
       _board_wlc_type = OEM_WLC_ON_BOARD_TYPE;

       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE );
       break;
       
    case PORT_MASK_HHL:
       _board_type = OEM_BOARD2_TYPE;
       _board_wlc_type = OEM_WLC_ON_BOARD_TYPE;
       
       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       break;
       
    case PORT_MASK_HHH:
       _board_type = OEM_BOARD3_TYPE;
       _board_wlc_type = OEM_WLC_ON_BOARD_TYPE;
       
       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE );
       break;
    default:
       break;
  }
  printk( KERN_ERR "OEM_board_judgement() : board_type = %d wlc_type = %d", _board_type,  _board_wlc_type);
}

/*===========================================================================
  FUNCTION OEM_get_board()

  PARAMETERS
    None.

  RETURN VALUE
    oem_board_type : Board Type
===========================================================================*/
oem_board_type OEM_get_board(void)
{
  return( _board_type );
}
EXPORT_SYMBOL(OEM_get_board);
/*===========================================================================
  FUNCTION OEM_get_wireless_charge()

  PARAMETERS
    None.

  RETURN VALUE
    oem_wireless_charge_type : Board Type
===========================================================================*/
oem_wireless_charge_type OEM_get_wireless_charge(void)
{
  return( _board_wlc_type );
}
EXPORT_SYMBOL(OEM_get_wireless_charge);
