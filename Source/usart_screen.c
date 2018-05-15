#include "usart_screen.h"

extern u32 time;
extern u8 usart_state;
screen_usart_t sut;
char screen_tx_buffer[20];
char screen_rx_buffer[20];
time_t rct_t;
                                              //与显示屏串口通信任务
void su_mission_polling(void){
	switch(sut.mission_state){
		
		case SCREEN_IDLE:
			
			break;
		
		case SCREEN_PENDING:
		//waiting time 
		
		if(sut.waitime>time)
			break;
		sut.mission_state=SCREEN_QUERY;
		
			break;
		case SCREEN_QUERY:
			sprintf(screen_tx_buffer,"rct1=%d",sut.index);
			TM_USART_DMA_Send(USART1, (uint8_t *)screen_tx_buffer,parpare_buffer());
			//set timer
			sut.timeout=time+SCREEN_TIMEOUT;
			sut.mission_state=SCREEN_POLLOING;
		
			break;
		
		case SCREEN_POLLOING:
			if(time>sut.timeout){
				sut.retry_count++;
				
				if(sut.retry_count>MAX_RETRY){
					sut.mission_state=SCREEN_FAILED;
					break;
				}
				sut.mission_state=SCREEN_PENDING;
				break;
			}
				
				if(TM_USART_Gets(USART1,screen_rx_buffer,5)){
					if(screen_rx_buffer[0]==SCREEN_ACK){

						sut.index++;
						sut.retry_count=0;
						sut.mission_state=SCREEN_PENDING;
					}
						
				}
					sut.waitime=time+SCREEN_WAITIME;
				if(sut.index>5){
					sut.mission_state=SCREEN_SUCCESS;
				}
					break;
				
		case SCREEN_FAILED:
			mission_failed_send(sut.current_mission);
			sut.index=0;
			sut.mission_state=SCREEN_IDLE;
			sut.retry_count=0;
			break;
		
		case SCREEN_SUCCESS:
			mission_success_send(sut.current_mission);
				sut.index=0;
				sut.mission_state=SCREEN_IDLE;
				sut.retry_count=0;
			break;
		
		default:
				sut.mission_state=SCREEN_IDLE;
		break;
			}
	}
	


u8 parpare_buffer(){
		u8 length;
		
		if(sut.index==0){
			if(rct_t.year2>0x10)
				sprintf(screen_tx_buffer,"rtc0=%x%x",rct_t.year1,rct_t.year2);
			else if(rct_t.year2>0x0)
				sprintf(screen_tx_buffer,"rtc0=%x0%x",rct_t.year1,rct_t.year2);
			else if(rct_t.year2==0)
				sprintf(screen_tx_buffer,"rtc0=%x00",rct_t.year1);
			length=9;
		}
		if(sut.index>0 && sut.index<6){
				sprintf(screen_tx_buffer,"rtc%d=%x",sut.index,rct_t.buf[sut.index-1]);
				if(rct_t.buf[sut.index-1]>=0x10)
					length=7;
				else
					length=6;
		}
			screen_tx_buffer[length++]=0xFF;
			screen_tx_buffer[length++]=0xFF;
			screen_tx_buffer[length++]=0xFF;
			return length;
}



