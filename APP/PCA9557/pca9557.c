#include "pca9557.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver_gpio.h"

#define PCA9557_I2C_MASTER_SCL_IO           GPIO_NUM_37       /*!< gpio number for I2C master clock */
#define PCA9557_I2C_MASTER_SDA_IO           GPIO_NUM_38      /*!< gpio number for I2C master data  */
#define PCA9557_I2C_MASTER_NUM              I2C_NUM_1    /*!< I2C port number for master dev */
#define PCA9557_I2C_MASTER_FREQ_HZ          100000  /*!< I2C master clock frequency */
#define PCA9557_I2C_MASTER_TX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define PCA9557_I2C_MASTER_RX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */



#include "driver_iic.h"

const static char *TAG_PCA9557 = "PCA9557";

#define IIC_ADDRESS 0x30
#define address_read  0x31
#define REGISTER_0_INPUT      0x00 // Register 0 - Input port register
#define REGISTER_1_OUTPUT     0x01 // Register 1 - Output port register
#define REGISTER_2_INVERSION  0x02 // Register 2 - Polarity inversion register
#define REGISTER_3_CONFIG     0x03 // Register 3 - Configuration register

typedef struct pca9557_io {
	uint8_t PIN_MOD_OUT;
	uint8_t PIN_MOD_IN;
} PCA9557_IO_MODE;
PCA9557_IO_MODE io_mode;

esp_err_t init_pca9557(void)
{
	uint8_t ret = 0;
	uint8_t data_io = 0x8C;  // 10001110 0x0c P2、P3、P7引脚设置为输入，其他设置为输出
	uint8_t data_inversion = 0x00; // 00000000 所有引脚都设置为不反转
	uint8_t data_out = 0xFF;

	io_mode.PIN_MOD_OUT = 0x00;

	ESP_LOGI(TAG_PCA9557, "init_pca9557\n");
	ret = dirver_i2c1_init();
	ret = driver_i2c1_master_transmit_buf(IIC_ADDRESS, REGISTER_3_CONFIG, &data_io, 1); // 设置引脚方向
	printf("%d\n", ret);
	ret = driver_i2c1_master_transmit_buf(IIC_ADDRESS, REGISTER_2_INVERSION, &data_inversion, 1); // 设置引脚反转 不反转
	printf("%d\n", ret);
	ret = driver_i2c1_master_transmit_buf(IIC_ADDRESS, REGISTER_1_OUTPUT, &data_out, 1); // 设置引脚输出电平
	printf("%d\n", ret);
	if(ret == ESP_OK) {
		ESP_LOGI(TAG_PCA9557, "init_pca9557 OK");
	}
	return ret;
}

esp_err_t updata_pca9557_io_input(void)
{
	uint8_t recv = driver_i2c1_master_receive_buf(IIC_ADDRESS, REGISTER_0_INPUT, &io_mode.PIN_MOD_OUT, 1);
	if(recv == ESP_OK) {
		return ESP_OK;
	}
	else {
		ESP_LOGE(TAG_PCA9557, "read_pca9557_io_input ERROR");
		return -1;
	}


}

uint8_t read_pca9557_io_input(uint8_t pin)
{
	if(io_mode.PIN_MOD_OUT & (1 << pin)){
		return 1;
	}
	else {
		return 0;
	}
}

esp_err_t set_pca9557_io_high(uint8_t pin)
{
	uint8_t ret = 0;
	io_mode.PIN_MOD_IN |= (1 << pin);
	ret = driver_i2c1_master_transmit_buf(IIC_ADDRESS, REGISTER_1_OUTPUT, &io_mode.PIN_MOD_IN, 1); // 设置引脚输出电平
	if(ret != ESP_OK) {
		ESP_LOGI(TAG_PCA9557, "set_pca9557_io_high OK");
	}
	return ret;
}

esp_err_t set_pca9557_io_low(uint8_t pin)
{
	uint8_t ret = 0;
	io_mode.PIN_MOD_IN &= ~(1 << pin);
	ret = driver_i2c1_master_transmit_buf(IIC_ADDRESS, REGISTER_1_OUTPUT, &io_mode.PIN_MOD_IN, 1); // 设置引脚输出电平
	if(ret != ESP_OK) {
		ESP_LOGI(TAG_PCA9557, "set_pca9557_io_high OK");
	}
	return ret;
}






/*******************************************************************************
* 名    称： bsp_Pca9557Init
* 功    能： PCA9577 I2C初始化函数
* 入口参数： 无
* 出口参数： 无
* 作　　者： Roger-WY
* 创建日期： 2021-05-08
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
esp_err_t bsp_Pca9557Init(void)
{
	int recv = 0;
    int i2c_master_port = PCA9557_I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PCA9557_I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = PCA9557_I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = PCA9557_I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    recv =  i2c_driver_install(i2c_master_port, conf.mode, PCA9557_I2C_MASTER_RX_BUF_DISABLE, PCA9557_I2C_MASTER_TX_BUF_DISABLE, 0);
    if(recv != ESP_OK){
    	ESP_LOGE(TAG_PCA9557, "bsp_Pca9557Init error");
    	return recv;
    }
    return ESP_OK;
}

/*******************************************************************************
* 名    称： bsp_Pca9557WriterReg
* 功    能： PCA9577 写寄存器
* 入口参数： u8I2cSlaveAddr : PCA9557的I2C地址
            u8Cmd : 命令寄存器
            u8Value : 写入寄存器的值
* 出口参数： esp_err_t
* 作　　者： Roger-WY
* 创建日期： 2021-05-08
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
esp_err_t bsp_Pca9557WriterReg(uint8_t u8I2cSlaveAddr,uint8_t u8Cmd,uint8_t u8Value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, u8I2cSlaveAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, u8Cmd, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, u8Value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(PCA9557_I2C_MASTER_NUM, cmd, 1000 );
    i2c_cmd_link_delete(cmd);
    return ret;

}

/*******************************************************************************
* 名    称： bsp_Pca9557ReadReg
* 功    能： PCA9577 读寄存器
* 入口参数： u8I2cSlaveAddr : PCA9557的I2C地址
            u8Cmd : 命令寄存器
            pBuff : 寄存器值的缓存数组指针
            u8Cnt : 读取的寄存器个数
* 出口参数： esp_err_t
* 作　　者： Roger-WY
* 创建日期： 2021-05-08
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
esp_err_t bsp_Pca9557ReadReg(uint8_t u8I2cSlaveAddr,uint8_t u8Cmd,uint8_t *pBuff,uint8_t u8Cnt)
{
    esp_err_t ret ;

    i2c_cmd_handle_t wr_cmd = i2c_cmd_link_create();
    i2c_master_start(wr_cmd);
    i2c_master_write_byte(wr_cmd, (u8I2cSlaveAddr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_write_byte(wr_cmd, u8Cmd, ACK_CHECK_EN);
    i2c_master_stop(wr_cmd);
    ret = i2c_master_cmd_begin(PCA9557_I2C_MASTER_NUM, wr_cmd, 1000 );
    i2c_cmd_link_delete(wr_cmd);
    if(ret != ESP_OK) {
    	if(ret == ESP_FAIL) {
			ESP_LOGE(TAG_PCA9557, "ESP_FAIL发送命令错误，slave没有ACK传输。:%d", ret);
    	}
		ESP_LOGE(TAG_PCA9557, "Pca9557 Read Reg error:%d", ret);
		return ret;
	}
//    vTaskDelay(30 / portTICK_RATE_MS);
    i2c_cmd_handle_t rd_cmd = i2c_cmd_link_create();
    i2c_master_start(rd_cmd);
    i2c_master_write_byte(rd_cmd, (u8I2cSlaveAddr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(rd_cmd, pBuff, u8Cnt, I2C_MASTER_LAST_NACK);
    i2c_master_stop(rd_cmd);
    ret = i2c_master_cmd_begin(PCA9557_I2C_MASTER_NUM, rd_cmd, 1000  );
    i2c_cmd_link_delete(rd_cmd);
    if(ret != ESP_OK) {
    	ESP_LOGE(TAG_PCA9557, "Pca9557 Read Reg error:%d", ret);
    	return ret;
    }
    return ESP_OK;

}


esp_err_t iic_read_(uint8_t u8I2cSlaveAddr,uint8_t u8Cmd,uint8_t *pBuff,uint8_t u8Cnt)
{
	esp_err_t ret ;
	uint8_t buf[2] = {u8I2cSlaveAddr, u8Cmd};
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write(cmd, buf, 2, ACK_CHECK_EN);
//	i2c_master_write_byte(cmd, u8I2cSlaveAddr, ACK_CHECK_EN);
//	i2c_master_write_byte(cmd, u8Cmd, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, pBuff, ACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(PCA9557_I2C_MASTER_NUM, cmd, 1000);
	i2c_cmd_link_delete(cmd);
	return ret;
}

/*******************************************************************************
* 名    称： bsp_PcaSetIoDirection
* 功    能： PCA9577 设置IO方向
* 入口参数： pinx : pin脚名称
            newMode : 输入或者输出
* 出口参数： esp_err_t
* 作　　者： Roger-WY
* 创建日期： 2021-05-08
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
esp_err_t bsp_PcaSetIoDirection(snPinName_t pinx,snPinMode_t newMode)
{
    esp_err_t ret ;
    uint8_t current_mode = 0;
    ret = bsp_Pca9557ReadReg(PCA9557_I2C_SLAVE_ADDR, PCA9557_CONTROL_REG_3, &current_mode, 1);
    if (ret != ESP_OK) {
    	if(ret == ESP_ERR_TIMEOUT) ESP_LOGE(TAG_PCA9557, "IIC 读写超时:%d", ret);
    	ESP_LOGE(TAG_PCA9557, "Pca Set Io Direction,Read Reg ERROR :%d", ret);
        return ret;
    }
    if(newMode == IO_OUTPUT)
    {
        current_mode &= ~(1 << pinx);
    }
    else
    {
        current_mode |= (1 << pinx);
    }
    ret = bsp_Pca9557WriterReg(PCA9557_I2C_SLAVE_ADDR, PCA9557_CONTROL_REG_3, current_mode);
    if(ret != ESP_OK) {
    	ESP_LOGE(TAG_PCA9557, "Pca9557 Set Io Direction error:%d", ret);
    	return ret;
    }
    return ESP_OK;
}
/*******************************************************************************
* 名    称： bsp_PcaSetIoStatus
* 功    能： PCA9577 设置IO状态
* 入口参数： pinx : pin脚名称
            snPinState_t : 高电平或者低电平
* 出口参数： esp_err_t
* 作　　者： Roger-WY
* 创建日期： 2021-05-08
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
esp_err_t bsp_PcaSetIoStatus(snPinName_t pinx,snPinState_t newState)
{
    esp_err_t ret ;
    uint8_t current_state = 0;
    ret = bsp_Pca9557ReadReg(PCA9557_I2C_SLAVE_ADDR, PCA9557_CONTROL_REG_1, &current_state, 1);
    if (ret != ESP_OK) {
    	ESP_LOGE(TAG_PCA9557, "Pca Set Io Status,Read Reg ERROR :%d", ret);
        return ret;
    }
    if(newState == IO_LOW)
    {
        current_state &= ~(1 << pinx);
    }
    else
    {
        current_state |= (1 << pinx);
    }
    ret = bsp_Pca9557WriterReg(PCA9557_I2C_SLAVE_ADDR, PCA9557_CONTROL_REG_1, current_state);
    return ret;

}

/*******************************************************************************
* 名    称： bsp_PcaSetInputPolarity
* 功    能： PCA9577 设置IO输入的极性是否翻转
* 入口参数： pinx : pin脚名称
            newPolarity : 极性是否翻转
* 出口参数： esp_err_t
* 作　　者： Roger-WY
* 创建日期： 2021-05-08
* 修    改：
* 修改日期：
* 备    注：默认bit0-bit3极性不翻转 bit4-bit7极性翻转
*******************************************************************************/
esp_err_t bsp_PcaSetInputPolarity(snPinName_t pinx,snPinPolarity_t newPolarity)
{
    esp_err_t ret ;
    uint8_t current_state = 0;
    ret = bsp_Pca9557ReadReg(PCA9557_I2C_SLAVE_ADDR, PCA9557_CONTROL_REG_2, &current_state, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    if(newPolarity == IO_NON_INVERTED)
    {
        current_state &= ~(1 << pinx);
    }
    else
    {
        current_state |= (1 << pinx);
    }
    ret = bsp_Pca9557WriterReg(PCA9557_I2C_SLAVE_ADDR, PCA9557_CONTROL_REG_2, current_state);
    return ret;

}

/*******************************************************************************
* 名    称： bsp_PcaGetIoStatus
* 功    能： PCA9577 设置IO输入的状态
* 入口参数： pinx : pin脚名称
* 出口参数： snPinState_t 引脚的电平(0/1)
* 作　　者： Roger-WY
* 创建日期： 2021-05-08
* 修    改：
* 修改日期：
* 备    注：注意是否在极性翻转寄存器中设置了翻转极性!!!
*******************************************************************************/
snPinState_t bsp_PcaGetIoStatus(snPinName_t pinx)
{
    esp_err_t ret ;
    uint8_t current_state = 0;
    ret = bsp_Pca9557ReadReg(PCA9557_I2C_SLAVE_ADDR, PCA9557_CONTROL_REG_0, &current_state, 1);

    if(ret == ESP_OK)
    {
        if(current_state & (1 << pinx))
        {
            return IO_HIGH;
        }
        else
        {
            return IO_LOW;
        }
    }
    else
    {
        return IO_UNKNOW;   //返回未知状态
    }
}



