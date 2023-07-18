#pragma once

#include "usart_interface.h"

#define AIM_ASSIST_DATA_SEND_FLOAT_NUM          ((uint8_t)11)
#define AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM       ((uint8_t)9)
#define AIM_ASSIST_DATA_SEND_SIZE               ((uint8_t)6 + 4 * AIM_ASSIST_DATA_SEND_FLOAT_NUM)
#define AIM_ASSIST_DATA_RECEIVE_SIZE            ((uint8_t)6 + 4 * AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM)


class AimAssist {
public:
    #pragma pack(1)
    struct AimAssistDataSendFrame {
        unsigned char m_head[2];
        unsigned char m_id;
        unsigned char m_length;
        float m_data[AIM_ASSIST_DATA_SEND_FLOAT_NUM];
        unsigned char m_tail[2];
    };
    struct AimAssistDataReceiveFrame {
        unsigned char m_head[2];
        unsigned char m_id;
        unsigned char m_length;
        float m_data_f[AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM];  
        unsigned char m_tail[2];
    };
    #pragma pack()
	
	unsigned char m_data_send_size;
	AimAssistDataSendFrame m_data_send_frame;
    AimAssistDataReceiveFrame m_data_receive_frame;

    Usart* usart_assist;

    bool balance_infantry_flag;
    float balance_infantry_num;
    bool m_enemy_1_res_flag;
    bool m_enemy_2_res_flag;
    bool m_enemy_3_res_flag;
    bool m_enemy_4_res_flag;
    bool m_enemy_5_res_flag;
    bool m_enemy_7_res_flag;
  
    uint16_t m_enemy_1_res_cnt;
    uint16_t m_enemy_2_res_cnt;
    uint16_t m_enemy_3_res_cnt;
    uint16_t m_enemy_4_res_cnt;
    uint16_t m_enemy_5_res_cnt;

    AimAssist(void){
		m_data_send_size = AIM_ASSIST_DATA_SEND_SIZE;
		
		m_data_send_frame.m_head[0] = 0x55;
        m_data_send_frame.m_head[1] = 0x00;
		m_data_send_frame.m_id = 0x00;
		m_data_send_frame.m_length = AIM_ASSIST_DATA_SEND_FLOAT_NUM;
        for (int i = 0; i < AIM_ASSIST_DATA_SEND_FLOAT_NUM; i++) {
            m_data_send_frame.m_data[i] = 0;
        }
		m_data_send_frame.m_tail[0] = 0x00;
        m_data_send_frame.m_tail[1] = 0xAA;

		m_data_receive_frame.m_head[0] = 0x55;
        m_data_receive_frame.m_head[1] = 0x00;
		m_data_receive_frame.m_id = 0x00;
		m_data_receive_frame.m_length = AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM;
        for (int i = 0; i < AIM_ASSIST_DATA_RECEIVE_FLOAT_NUM; i++) {
            m_data_receive_frame.m_data_f[i] = 0;
        }
		m_data_receive_frame.m_tail[0] = 0x00;
        m_data_receive_frame.m_tail[1] = 0xAA;

        balance_infantry_flag = false;
        balance_infantry_num = 0;

        m_enemy_1_res_flag = false;
        m_enemy_2_res_flag = false;
        m_enemy_3_res_flag = false;
        m_enemy_4_res_flag = false;
        m_enemy_5_res_flag = false;
        m_enemy_7_res_flag = true;
        
        m_enemy_1_res_cnt = 0;
        m_enemy_2_res_cnt = 0;
        m_enemy_3_res_cnt = 0;
        m_enemy_4_res_cnt = 0;
        m_enemy_5_res_cnt = 0;

        usart_assist = new Usart();
    };

    void SetUartHandle(USART_TypeDef* USARTx);
    void SendData(void);

private:
};