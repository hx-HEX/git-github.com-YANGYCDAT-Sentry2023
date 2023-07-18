#include "user_task.h"
#include "verify.h"

DECLARE_RM_OS_TASK()
DECLARE_RM_OS_FLAG()

void LaunchAllTasks(void){
    taskENTER_CRITICAL();

		CREATE_OS_TASK(LedTask);
		CREATE_OS_TASK(DataVisualTask);
		CREATE_OS_TASK(RobotControlTask);
		CREATE_OS_TASK(UsartDecodeTask);
        CREATE_OS_TASK(DataCommunicateTask);

		USARTHandler = xEventGroupCreate();//创建事件组，返回句柄

    taskEXIT_CRITICAL();
    vTaskStartScheduler();
}



/**
 * @brief communication task
 *
 * @param NULL
 */
void DataCommunicateTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(DataCommunicate_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		G_gimbal.m_data_send_frame.m_fdata[0] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
		G_gimbal.m_data_send_frame.m_fdata[1] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed->GetFilterOutput();
		#ifdef FIRST_FITCH_ANGLE_ENCODER_FEEDBACK
		G_gimbal.m_data_send_frame.m_fdata[2] = -G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
		#endif
		#ifdef FIRST_FITCH_ANGLE_IMU_FEEDBACK
		G_gimbal.m_data_send_frame.m_fdata[2] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.pitch;
		#endif
		#ifdef FIRST_FITCH_SPEED_IMU_FEEDBACK
		G_gimbal.m_data_send_frame.m_fdata[3] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_y->GetFilterOutput() * RADIANODEGREES;
		#endif
		#ifdef FIRST_FITCH_SPEED_ENCODER_FEEDBACK
		G_gimbal.m_data_send_frame.m_fdata[3] = -G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_speed->GetFilterOutput();
		#endif
		G_gimbal.m_data_send_frame.m_fdata[4] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
		G_gimbal.m_data_send_frame.m_fdata[5] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z->GetFilterOutput() * RADIANODEGREES;
		G_gimbal.m_data_send_frame.m_fdata[6] = (G_gun_shift.m_target_angle == GUNSHIFT_ANGLE_1)?1:-1;

		Append_CRC16_Check_Sum((uint8_t*)&G_gimbal.m_data_send_frame,GIMBAL_DATA_SEND_SIZE);
		G_gimbal.SendData();
		
		G_system_monitor.DataCommunicateTask_cnt++; // Statistic task execution times
		G_system_monitor.DataCommunicateTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}


/**
 * @brief Data visual task
 *
 * @param NULL
 */
void DataVisualTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(DataVisualize_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		// the ID of mdata should be less than 15
		#ifdef GIMBAL_DEBUG
		G_VisualizeGimbalData();
		#endif
		
		G_vofa.m_data_send_frame.m_data[15] = G_system_monitor.SysTickTime;

		G_vofa.SendData();

		G_system_monitor.DataVisualizeTask_cnt++; // Statistic task execution times
		G_system_monitor.DataVisualizeTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



/**
 * @brief LED task
 *
 * @param NULL
 */
void LedTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(Led_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();
		
		G_led.ToggleGreen();

		if (G_system_monitor.UART6_rx_fps > 900) {
			G_led.ToggleBlue();
		}

		G_system_monitor.LedTask_cnt++; // Statistic task execution times
		G_system_monitor.LedTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



/**
 * @brief Robot control task
 *
 * @param NULL
 */
void RobotControlTask(void *pvParameters) {
	static u32 TaskStartTime;
	const TickType_t RouteTimes = pdMS_TO_TICKS(RobotControl_TASK_CYCLE);

	while(1)
	{
		TaskStartTime = TIME();

		// Update the control mode
		ControlModeUpdate();
		// Update the robot state
		RobotStatesUpdate();
		// Update the robot targets
		RobotTargetsUpdate();
		// Execute the robot control
		RobotControlExecute();

		G_system_monitor.RobotControlTask_cnt++; // Statistic task execution times
		G_system_monitor.RobotControlTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
		
		vTaskDelay(RouteTimes);
	}
}



void UsartDecodeTask(void *pvParameters) {
	static u32 TaskStartTime;
	EventBits_t EventValue=0;

	while(1)
	{
		EventValue = GetEventGroupFlag(USART);
		TaskStartTime = TIME();

		UART_Decode(EventValue);

		G_system_monitor.UsartDecodeTask_ExecuteTime = TIME() - TaskStartTime; // Caculate the execution time of this task
	}
}