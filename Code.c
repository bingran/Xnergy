//put your definition here

#define MINIMUN_CURRENT 0.1
#define CURRENT_REFERENCE 1
#deifne VOLTAGE_REFERENCE 4.5

typedef enum
{
	chargerIdle = 0,
	chargerConstantVoltage,
	chargerConstantCurrent
} chargerState;


typedef enum
{
	chargerNetworkInitialization = 0,
	chargerNetworkPreOperational,
	chargerNetworkOperational
} chargerNetworkState;

//CAN struct example
typedef struct {
	uint8_t Data[8];
	uint16_t Length;
	uint32_t ID;
} CAN_msg_typedef;



bool bEnable_command = false;

/* constants for PID */
const float flVoltage_Kp = 0.01;
const float flVoltage_Ki = 0.01;
const float flVoltage_Kd = 0.001;

float flVoltage_feedback = 0;
float flVoltage_target = 0;
float flVoltage_PID_error = 0;
float flVoltage_PID_previous_error = 0;
uint8_t u8Voltage_PID_p = 0;    
uint8_t u8Voltage_PID_i = 0; 
uint8_t u8Voltage_PID_d = 0; 

/* constants for PID */
const float flCurrent_Kp = 0.01;
const float flCurrent_Ki = 0.01;
const float flCurrent_Kd = 0.001;

float flCurrent_feedback = 0;
float flCurrent_target = 0;
float flCurrent_PID_error = 0;
float flCurrent_PID_previous_error = 0;
uint8_t u8Current_PID_p = 0;    
uint8_t u8Current_PID_i = 0; 
uint8_t u8Current_PID_d = 0; 

uint16_t PIDTimeOut = 0 ;

uint16_t canMsgHearBeatTxTimeOut = 0 ;
uint16_t canMsgChargingCmdRxTimeOut = 0 ;

uint16_t canMsgChargingStatusTxTimeOut = 0 ;

uint16_t u16temp = 0;

bool bCanMsgHeartBeatTx = false;
bool bCanMsgChargingStatusTx = false;

bool bCanMsgChargingCmdRx = false;

bool bCanWriteInProgress = false;

CAN_msg_typedef Can_tx;
CAN_msg_typedef Can_rx;

chargerState eChargerState = chargerIdle;
chargerNetworkState echargerNetworkState = chargerNetworkInitialization;


void Initialization(void){

}

void control_routine(void){ 
	time_ms++; //assume INT frequency is 1kHz, for timing purpose	
}
void main_state_machine(void){
	switch (eChargerState){
		case chargerIdle:{
			
			flCurrent_target = 0;
			flVoltage_target = 0;
			
			if(true == bEnable_command){
				eChargerState = chargerConstantVoltage;
			}
			break;
		}
		case chargerConstantVoltage:{
			
			flCurrent_target = CURRENT_REFERENCE;
			flVoltage_target = VOLTAGE_REFERENCE;
			
			if(VOLTAGE_REFERENCE <= flVoltage_feedback){
				eChargerState = chargerConstantCurrent;
			}
			break;
		}
		case chargerConstantCurrent:{
			
			flCurrent_target = MINIMUN_CURRENT;
			flVoltage_target = VOLTAGE_REFERENCE;
			
			if(MINIMUN_CURRENT >= flCurrent_feedback){
				bEnable_command = false;
				eChargerState = chargerIdle;
			}
			break;
		}
		default:{
			eChargerState = chargerIdle;
			break;
		}
	}
}	

void CAN_write_handler(void){
	bCanWriteInProgress = false;
}

void CAN_read_handler(void){
	if(true == CAN_read(&Can_rx){
		if(0x00000201 == Can_rx.ID)&&(0x04 == Can_rx.Length)){
			canMsgChargingTimeOut = 5000;
			bCanMsgChargingCmdRx = true;
			u16temp = Can_rx.Data[0];
			u16temp <<= 8;
			u16temp |= Can_rx.Data[1];
			flVoltage_feedback = ((float)u16temp)/10.0;
			u16temp = Can_rx.Data[2];
			u16temp <<= 8;
			u16temp |= Can_rx.Data[3];
			flCurrent_feedback = ((float)u16temp)/10.0;
			bEnable_command = Can_rx.Data[4];
			
		}
	}
}

void network_management(void){
	switch (echargerNetworkState){
		case chargerNetworkInitialization:{
			if(true == Intialized){
				echargerNetworkState = chargerNetworkPreOperational;
			}
			break;
		}
		case chargerNetworkPreOperational:{
			if(true == bCanMsgChargingCmdRx){
				echargerNetworkState = chargerNetworkOperational;
			}

			break;
		}
		case chargerNetworkOperational:{
			if(false == bCanMsgChargingCmdRx){
				echargerNetworkState = chargerNetworkPreOperational;
			}
			break;
		}
		default:{
			echargerNetworkState = chargerNetworkInitialization;
			break;
		}
	}
}

void main(void){
	Initialization();
	PieVectTable.EPWM1_INT = &control_routine;
	while(true){
		main_state_machine();
		network_management();
		if( 0 != time_ms){
			canMsgHearBeatTxTimeOut++;
			PIDTimeOut++;
			if( 1000 < canMsgHearBeatTxTimeOut){
				canMsgHearBeatTxTimeOut = 0;
				bCanMsgHeartBeatTx = true;
			}
			
			if(true  == bCanMsgChargingCmdRx){
				canMsgChargingCmdRxTimeOut--;
				bCanMsgChargingCmdRx = false;
			}
			
			if(chargerNetworkOperational == echargerNetworkState){
				canMsgChargingStatusTxTimeOut++;
				if(200> canMsgChargingStatusTxTimeOut)
				{
					canMsgChargingStatusTxTimeOut = 0;
					bCanMsgChargingStatusTx = 0;
				}
			}
			
			if(false == bCanWriteInProgress){
				if(true == bCanMsgHeartBeatTx){
					bCanMsgHeartBeatTx = false;
					Can_tx.Data[0] = eChargerState;
					Can_tx.Data[1] = 0x00;
					Can_tx.Data[2] = 0x00;
					Can_tx.Data[3] = 0x00;
					Can_tx.Data[4] = 0x00;
					Can_tx.Data[5] = 0x00;
					Can_tx.Data[6] = 0x00;
					Can_tx.Data[7] = 0x00;
					Can_tx.Length = 0x01;
					Can_tx.ID  = 0x00000701;
					bCanWriteInProgress = true;
				}
				else if( true == bCanMsgChargingStatusTx){
					bCanMsgChargingStatusTx = false;
					u16temp = (uint16_t)(flVoltage_feedback*10);
					Can_tx.Data[0] = u16temp>>8;
					Can_tx.Data[1] = u16temp;
					u16temp = (uint16_t)(flCurrent_feedback*10);
					Can_tx.Data[2] = u16temp>>8;
					Can_tx.Data[3] = u16temp;
					if(chargerNetworkOperational != echargerNetworkState0{	
						Can_tx.Data[4] = 0x01;
					}
					else{	
						Can_tx.Data[4] = 0x02;
					}
					Can_tx.Data[5] = 0x00;
					Can_tx.Data[6] = 0x00;
					Can_tx.Data[7] = 0x00;
					Can_tx.Length = 0x01;
					Can_tx.ID  = 0x00000181;
					bCanWriteInProgress = true;
				}
			}
			
			if(200 < PIDTimeOut){
				PIDTimeOut = 0 ;
				flVoltage_PID_error = flVoltage_target - flVoltage_feedback;

				u8Voltage_PID_p = flVoltage_Kp * flVoltage_PID_error;
				u8Voltage_PID_i = u8Voltage_PID_i + (flVoltage_Ki * flVoltage_PID_error);
				u8Voltage_PID_d = flVoltage_Kd*((flVoltage_PID_error - flVoltage_PID_previous_error)/200);/* assume 200ms */

				voltagePID_value = u8Voltage_PID_p + u8Voltage_PID_i + u8Voltage_PID_d;
				/*Assume PWM range between 0 and 255*/
				if(voltageID_value < 0){    
					voltageID_value = 0;    
				}
				else if(voltagePID_value > 255){    
					voltagePID_value = 255;  
				}
				updateVoltagePWM(255-voltagePID_value);
				flVoltage_PID_previous_error = flVoltage_PID_error;     
				
				
				flCurrent_PID_error = flCurrent_target - flCurrent_feedback;

				u8Current_PID_p = flCurrent_Kp * flCurrent_PID_error;
				u8Current_PID_i = u8Current_PID_i + (flCurrent_Ki * flCurrent_PID_error);
				u8Current_PID_d = flCurrent_Kd*((flCurrent_PID_error - flCurrent_PID_previous_error)/100);/* assume 100ms */

				CurrentPID_value = u8Current_PID_p + u8Current_PID_i + u8Current_PID_d;
				/*Assume PWM range between 0 and 255*/
				if(CurrentID_value < 0){    
					CurrentID_value = 0;    
				}
				else if(CurrentPID_value > 255){    
					CurrentPID_value = 255;  
				}
				updateCurrentPWM(255-CurrentPID_value);
				flCurrent_PID_previous_error = flCurrent_PID_error;
			}
		}
	}
}