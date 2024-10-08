
/*
 * Copyright (c) 2020 panrui <https://github.com/Prry/rtt-bmp180>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-12     panrui      the first version
 */
 
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include "bmp180.h"

#define PKG_USING_BMP180

#ifdef PKG_USING_BMP180

#define BMP180_ADDR				0x77 	/* i2c slave address */

#define BMP_REG_CHIP_ID			0xD0	
#define BMP_REG_RESET			0xE0
#define BMP_REG_CTRL_ADDR		0xF4
#define BMP_REG_AD_MSB			0xF6
#define BMP_REG_AD_LSB			0xF7
#define BMP_REG_AD_XLSB			0xF8
#define BMS_CAL_AC1				0xAA

#define BMP_REG_CTRL_TEMP		0x2E
#define BMP_REG_CTRL_POSS0		0x34
#define BMP_REG_CTRL_POSS1		0x74
#define BMP_REG_CTRL_POSS2		0xB4
#define BMP_REG_CTRL_POSS3		0xF4
#define BMP_REG_RESET_VALUE		0xB6
		
#define	BMP180_I2C_BUS			"i2c1"		/* i2c linked */
#define BMP180_DEVICE_NAME		"bmp180"	/* register device name */


/* bmp clalc param */
struct bmp180_calc
{
	short ac1;
	short ac2;
	short ac3;
	short b1;
	short b2;
	short mb;
	short mc;
	short md;
	unsigned short ac4;
	unsigned short ac5;
	unsigned short ac6;	
};

/* bmp180 private data */
struct bmp180_dev
{
	struct bmp180_calc calc_param;
	struct rt_i2c_bus_device *i2c_bus;	/* linked i2c bus */
};

static rt_err_t  bmp180_read_regs(rt_sensor_t psensor, rt_uint8_t reg, rt_uint8_t *data, rt_uint8_t data_size)
{
    struct rt_i2c_msg msg[2];
	struct bmp180_dev *dev = RT_NULL;
	struct rt_i2c_bus_device *i2c_bus = RT_NULL;
	rt_uint32_t slave_addr = 0;
	
	slave_addr = (rt_uint32_t)psensor->config.intf.user_data;	/* get i2c slave address */
	dev = (struct bmp180_dev*)psensor->parent.user_data;/* bmp180 private data */
	i2c_bus = (struct rt_i2c_bus_device*)dev->i2c_bus; /* get i2c bus device */
	
    msg[0].addr  = (rt_uint8_t)slave_addr;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = &reg;
    msg[1].addr  = (rt_uint8_t)slave_addr;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = data_size;
    msg[1].buf   = data;

    if(rt_i2c_transfer(i2c_bus, msg, 2) == 2)
	{
        return RT_EOK;
    }
    else
    {
	  	LOG_E("i2c bus read failed!\r\n");
        return -RT_ERROR;
    }
}

static rt_err_t  bmp180_write_regs(rt_sensor_t psensor, rt_uint8_t reg, rt_uint8_t *data, rt_uint8_t data_size)
{
    struct rt_i2c_msg msg[2];
	struct bmp180_dev *dev = RT_NULL;
	struct rt_i2c_bus_device *i2c_bus;
	rt_uint32_t slave_addr = 0;

	slave_addr = (rt_uint32_t)psensor->config.intf.user_data;
	dev = (struct bmp180_dev*)psensor->parent.user_data;
	i2c_bus = (struct rt_i2c_bus_device*)dev->i2c_bus;  
	
    msg[0].addr		= (rt_uint8_t)slave_addr;
    msg[0].flags	= RT_I2C_WR;
    msg[0].len   	= 1;
    msg[0].buf   	= &reg;
    msg[1].addr  	= (rt_uint8_t)slave_addr;
    msg[1].flags	= RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len   	= data_size;
    msg[1].buf   	= data;
    if(rt_i2c_transfer(i2c_bus, msg, 2) == 2)
	{
        return RT_EOK;
    }
    else
    {
	  	LOG_E("i2c bus write failed!\r\n");
        return -RT_ERROR;
    }
}

static rt_err_t bmp180_write_reg(rt_sensor_t psensor, uint8_t reg, uint8_t data)
{
	return bmp180_write_regs(psensor, reg, &data, 1);
}

static long bmp180_read_ut(rt_sensor_t psensor)
{
	uint8_t buf[2] = {0};
	long   data = 0;
	
	bmp180_write_reg(psensor, BMP_REG_CTRL_ADDR, BMP_REG_CTRL_TEMP);
	rt_thread_delay(1);	/* max conversion time: 4.5ms */
	bmp180_read_regs(psensor, BMP_REG_AD_MSB, buf, 2);
	data = (buf[0]<<8) | buf[1];
	
	return data;
}

static long bmp180_read_up(rt_sensor_t psensor)
{
	uint8_t buf[2] = {0};
	long   data = 0;
	
	bmp180_write_reg(psensor, BMP_REG_CTRL_ADDR, BMP_REG_CTRL_POSS0);
	rt_thread_delay(1);	/* max conversion time: 4.5ms */
	bmp180_read_regs(psensor, BMP_REG_AD_MSB, buf, 2);
	data = (buf[0]<<8) | buf[1];
	
	return data;
}

static rt_size_t bmp180_polling_get_data(rt_sensor_t psensor, struct rt_sensor_data *sensor_data)
{
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;
	short temperature=0;
	long ut,up,pressure=0;
	struct bmp180_dev *dev = RT_NULL;
	struct bmp180_calc *param = RT_NULL;
	
	ut = bmp180_read_ut(psensor);
	up = bmp180_read_up(psensor);
	
	
	dev = (struct bmp180_dev*)psensor->parent.user_data;/* bmp180 private data */
	param = &dev->calc_param;	/* get calc param */
	
	/* temperature calc */
	x1 = (((long)ut - (long)param->ac6)*(long)param->ac5) >> 15;
  	x2 = ((long)param->mc << 11) / (x1 + param->md);
  	b5 = x1 + x2;
  	temperature = ((b5 + 8) >> 4);

	/* pressure calc */
	b6 = b5 - 4000;
	x1 = (param->b2 * (b6 * b6)>>12)>>11;
	x2 = (param->ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((long)param->ac1)*4 + x3)<<0) + 2)>>2;
	
	x1 = (param->ac3 * b6)>>13;
	x2 = (param->b1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (param->ac4 * (unsigned long)(x3 + 32768))>>15;
	b7 = ((unsigned long)(up - b3) * (50000>>0));
	if (b7 < 0x80000000)
	{
		p = (b7<<1)/b4;
	}
	else
	{
		p = (b7/b4)<<1;
	}
	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	pressure = p+((x1 + x2 + 3791)>>4);	
	
	if(psensor->info.type == RT_SENSOR_CLASS_BARO)
	{/* actual barometric */
	  	sensor_data->type = RT_SENSOR_CLASS_BARO;
		sensor_data->data.baro = pressure;
		sensor_data->timestamp = rt_sensor_get_ts();
	}
	else if(psensor->info.type == RT_SENSOR_CLASS_TEMP)
	{/* actual temperature */
		sensor_data->type = RT_SENSOR_CLASS_TEMP;
		sensor_data->data.temp = temperature;
		sensor_data->timestamp = rt_sensor_get_ts();
	}
	else
	{
		return 0;
	}
	
    return 1;
}

static RT_SIZE_TYPE bmp180_fetch_data(struct rt_sensor_device *psensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);
	RT_ASSERT(psensor);
	
	//if(psensor->parent.open_flag & RT_DEVICE_FLAG_RDONLY)
	if(psensor->config.mode == RT_SENSOR_MODE_POLLING)
	{
        return bmp180_polling_get_data(psensor, buf);
    }

    return 0;
}

static rt_err_t bmp180_control(struct rt_sensor_device *psensor, int cmd, void *args)
{
	rt_err_t	ret = RT_EOK;
    rt_uint8_t 	*chip_id;
	
    RT_ASSERT(psensor);

    switch (cmd)
    {
    	/* read bmp180 id */
        case RT_SENSOR_CTRL_GET_ID:
		  	chip_id = (rt_uint8_t*)args;
	       	ret = bmp180_read_regs(psensor, BMP_REG_CHIP_ID, chip_id, 1);
        break;

        default:
        break;
	}
    return ret;
}

static struct rt_sensor_ops bmp180_ops =
{
    bmp180_fetch_data,
    bmp180_control,
};

int rt_hw_bmp180_init(const char *name, struct rt_sensor_config *cfg)
{
  	rt_err_t ret = RT_EOK;
	rt_sensor_t sensor_baro = RT_NULL, sensor_temp = RT_NULL;
    struct rt_sensor_module *module = RT_NULL;
	struct bmp180_dev 		*bmp180 = RT_NULL;
    uint8_t bmbuf[22] = {0};
	
	bmp180 = rt_calloc(1, sizeof(struct bmp180_dev));
	if(bmp180 == RT_NULL)
	{
	  	LOG_E("malloc memory failed\r\n");
		ret = -RT_ERROR;
		goto __exit;
	}
	
    bmp180->i2c_bus = rt_i2c_bus_device_find(cfg->intf.dev_name);
    if(bmp180->i2c_bus == RT_NULL)
    {
        LOG_E("i2c bus device %s not found!\r\n", cfg->intf.dev_name);
		ret = -RT_ERROR;
		goto __exit;
    }	
	
	module = rt_calloc(1, sizeof(struct rt_sensor_device));
    if(module == RT_NULL)
	{
	  	LOG_E("malloc memory failed\r\n");
	  	ret = -RT_ERROR;
		goto __exit;
	}
	module->sen[0] = sensor_baro;
    module->sen[1] = sensor_temp;
    module->sen_num = 2;
	
	/*  barometric pressure sensor register */
    {
        sensor_baro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_baro == RT_NULL)
		{
		  	goto __exit;
		}
		rt_memset(sensor_baro, 0x0, sizeof(struct rt_sensor_device));
        sensor_baro->info.type       = RT_SENSOR_CLASS_BARO;
        sensor_baro->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_baro->info.model      = "bmp180_baro";
        sensor_baro->info.unit       = RT_SENSOR_UNIT_PA;
        sensor_baro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_baro->info.range_max  = 110000;	/* 1Pa */
        sensor_baro->info.range_min  = 30000;
        sensor_baro->info.period_min = 100;	/* read ten times in 1 second */

        rt_memcpy(&sensor_baro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_baro->ops = &bmp180_ops;
        sensor_baro->module = module;
        
        ret = rt_hw_sensor_register(sensor_baro, name, RT_DEVICE_FLAG_RDWR, (void*)bmp180);
        if (ret != RT_EOK)
        {
            LOG_E("device register err code: %d", ret);
            goto __exit;
        }
    }
    /* temperature sensor register */
    {
        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_temp == RT_NULL)
		{
			goto __exit;
		}
		rt_memset(sensor_temp, 0x0, sizeof(struct rt_sensor_device));
        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_temp->info.model      = "bmp180_temp";
        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_baro->info.range_max  = 850;	/* 0.1C */
        sensor_baro->info.range_min  = -400;
        sensor_temp->info.period_min = 100;	/* read ten times in 1 second */

        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
        sensor_temp->ops = &bmp180_ops;
        sensor_temp->module = module;
        
        ret = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, (void*)bmp180);
        if (ret != RT_EOK)
        {
            LOG_E("device register err code: %d", ret);
            goto __exit;
        }
    }
   
	/* bmp180 read calc param */
	ret = bmp180_read_regs(sensor_baro, BMS_CAL_AC1, bmbuf, 22);
	if(ret == RT_EOK)
	{
		bmp180->calc_param.ac1 = (bmbuf[0]<<8)|bmbuf[1];
		bmp180->calc_param.ac2 = (bmbuf[2]<<8)|bmbuf[3];
		bmp180->calc_param.ac3 = (bmbuf[4]<<8)|bmbuf[5];
		bmp180->calc_param.ac4 = (bmbuf[6]<<8)|bmbuf[7];
		bmp180->calc_param.ac5 = (bmbuf[8]<<8)|bmbuf[9];
		bmp180->calc_param.ac6 = (bmbuf[10]<<8)|bmbuf[11];
		bmp180->calc_param.b1 = (bmbuf[12]<<8)|bmbuf[13];
		bmp180->calc_param.b2 = (bmbuf[14]<<8)|bmbuf[15];
		bmp180->calc_param.mb = (bmbuf[16]<<8)|bmbuf[17];
		bmp180->calc_param.mc = (bmbuf[18]<<8)|bmbuf[19];
		bmp180->calc_param.md = (bmbuf[20]<<8)|bmbuf[21];
	}
	else
	{
		LOG_E("bmp180 read calc param failed\r\n");
		goto __exit;
	}
    return RT_EOK;

__exit:
  	if(sensor_baro)
	{
		rt_free(sensor_baro);
	}
	
	if(sensor_temp)
	{
		rt_free(sensor_temp);
	}
	
    if(module)
	{
	 	rt_free(module);
	}
	
	if (bmp180)
	{
		rt_free(bmp180);
	}
    return ret;
}

#endif /* PKG_USING_BMP180 */
