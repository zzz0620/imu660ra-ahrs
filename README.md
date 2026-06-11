# HPM5361
# 需要使用startgui.exe进行CMAKE生成项目；使用V8.24版本Segger编译，

## 关于MF7015的使用
```c
linkong_motor_t motor;

linkongMotorInit(&motor, 1, 1, 10);
    
linkongMotorEnable(&motor);
vTaskDelay(3000);
while(1)
{
linkongMotorSetPositionWithSpeedLimit(&motor, 90.0f,10);
vTaskDelay(3000);
linkongMotorSetPositionWithSpeedLimit(&motor, 0.0f,10);
vTaskDelay(3000);
}
```

## 关于达妙的使用

1. 使用rfl的dev的pvt模式
```c
rfl_motor_s motor4310;
rfl_motor_config_s config = {0};
rflMotorGetDefaultConfig(&config, RFL_MOTOR_DM_J8009_2EC, RFL_MOTOR_CONTROLLER_DAMIAO);
config.control_period_factor = 1.0f;

config.is_reversed = true;
rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_RADIAN, 12.5);
rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_RADIAN, -12.5);
config.can_ordinal = 1;
config.master_can_id = 0x02;
config.slave_can_id = 0x03;
config.max_speed = 1;

rflMotorInit(&motor4310, &config);
vTaskDelay(3000);

rflMotorSetMode(&motor4310,  RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
vTaskDelay(3000);

while(1)
{
flag_*=-1;
if(flag_==-1)
rflMotorSetAngle(&motor4310, RFL_ANGLE_FORMAT_RADIAN,1.5);
else if(flag_==1)
rflMotorSetAngle(&motor4310, RFL_ANGLE_FORMAT_RADIAN,0);

rflMotorUpdateStatus(&motor4310);
rflMotorUpdateControl(&motor4310);
rflMotorExecuteControl(&motor4310);

vTaskDelay(3000);
}
```

2. 使用rfl的rfl2.0的多种模式
```c
static void damiao_adapter_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    uint8_t tx_data[8] = {0};
    memcpy(tx_data, data, (len < 8) ? len : 8);
    rflCanSendData(1, id, tx_data);
}

static void damiao_adapter_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}


static rfl_damiao_motor_config_t motor_config = {
    .type = RFL_DM_MOTOR_J4310_2EC, 
    .motor_id = 0x03,              
    .p_max = 12.5f,                
    .v_max = 30.0f,               
    .t_max = 10.0f,              
    .can_send = damiao_adapter_can_send,
    .delay_ms = damiao_adapter_delay_ms,
};

static rfl_damiao_motor_t motor;

#define MIT 0
#define POS 1
#define SPD 2
#define PSI 3
#define TEST_dowdow MIT

static void can_rx_callback(uint8_t *data) {
    
    rflDmMotorUpdateStatus(&motor, data);
}

void arm_task(void *pvParameters)
{
    while (!INS_init_finished)
    {
    vTaskDelay(8);
    }

    rfl_err_t ret;
    TickType_t last_wake_time;
#if TEST_dowdow == POS
    rflCanRxMessageBoxAddId(1, 0x02);  
    rflCanRxMessageBoxAddRxCallbackFunc(1, 0x02, can_rx_callback);
    
    ret = rflDmMotorInit(&motor, &motor_config);
    
    ret = rflDmMotorEnable(&motor, RFL_DM_MOTOR_MODE_POS);

    damiao_adapter_delay_ms(100);
    
    last_wake_time = xTaskGetTickCount();
    
    while (1) {

        ret = rflDmMotorPosControl(&motor, 1.5f, 2.0f);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(3000));
        
        ret = rflDmMotorPosControl(&motor, 0.0f, 2.0f);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(3000));
        
    }
#elif TEST_dowdow == SPD
    rflCanRxMessageBoxAddId(1, 0x02);  
    rflCanRxMessageBoxAddRxCallbackFunc(1, 0x02, can_rx_callback);
    
    ret = rflDmMotorInit(&motor, &motor_config);
    
    ret = rflDmMotorEnable(&motor, RFL_DM_MOTOR_MODE_SPD);

    damiao_adapter_delay_ms(100);
    
    last_wake_time = xTaskGetTickCount();
    
    while (1) {

        ret = rflDmMotorSpdControl(&motor, 1.0f);
        static uint32_t counter = 0;
        if (counter++ % 10 == 0) { 
              printf("状态 - 位置: %.3f rad, 速度: %.3f rad/s, 扭矩: %.3f Nm\n",
                 motor.angle, motor.speed, motor.torque);
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));    
        }
#elif TEST_dowdow == MIT
    rflCanRxMessageBoxAddId(1, 0x02);  
    rflCanRxMessageBoxAddRxCallbackFunc(1, 0x02, can_rx_callback);
    
    ret = rflDmMotorInit(&motor, &motor_config);
    
    ret = rflDmMotorEnable(&motor, RFL_DM_MOTOR_MODE_MIT);

    damiao_adapter_delay_ms(100);

    last_wake_time = xTaskGetTickCount();
    
    while (1) {
        static uint8_t phase = 0;
        static uint32_t loop_count = 0;
        
        float pos = 0.0f;      
        float vel = 0.0f;      
        float kp = 50.0f;     
        float kd = 0.5f;       
        float torque = 0.0f;   
        
        if (loop_count % 256 == 0) {  
            phase = (phase + 1) % 4;
        }
        
        switch (phase) {
            case 0: 
                pos = 0.0f;     
                vel = 1.0f;     
                kp = 20.0f;    
                kd = 0.7f;      
                torque = 0.0f;
                //printf("MIT纯位置控制: pos=%.2f rad\n", pos);
                break;
                
            case 1: 
                pos = 0.0f;
                vel = 2.0f;     
                kp = 0.0f;     
                kd = 2.0f;     
                torque = 0.0f;
                //printf("MIT纯速度控制: vel=%.2f rad/s\n", vel);
                break;
                
            case 2: 
                pos = 0.0f;
                vel = 0.0f;
                kp = 0.0f;
                kd = 0.0f;
                torque = 0.5f; 
                //printf("MIT纯扭矩控制: torque=%.2f Nm\n", torque);
                break;
                
            case 3: 
                pos = 0.0f;     
                vel = 0.0f;
                kp = 80.0f;
                kd = 1.5f;
                torque = 0.2f;  
                //printf("MIT位置+前馈控制\n");
                break;
        }
        
        ret = rflDmMotorMitControl(&motor, pos, vel, kp, kd, torque);
        
        if (loop_count % 10 == 0) { 
            printf("反馈 - 位置: %.3f rad, 速度: %.3f rad/s, 扭矩: %.3f Nm\n",
                   motor.angle, motor.speed, motor.torque);
        }
        
        loop_count++;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));  
    }
#endif
}
```
