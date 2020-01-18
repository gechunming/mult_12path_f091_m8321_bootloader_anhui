GPB11 			 --->    EINT1  		 --->  lockstatus detect
GPB5,GPB6  		 --->   		 		 --->  I2C for bmp180
GPA9,GPA10		 --->           		 ---> UART1 for BC95
GPA3,GPA4		 --->          			 ---> LED or beep
GPB13,GPB14		 --->    EINT13,EINT14   ---> User Key
GPB2			 --->    EINT2			 ---> Vibrate detect
GPB1			 --->  					 ---> Unlock pin
GPA8			 --->                    ---> PWM for beep
GPB0 			 --->   	             ---> Moto Driver enable
GPB1,GPB10       ---> 					 ---> Moto rotate pin
GPA0 			 --->   				 ---> for battery adc
GPA5			 ---> 					 ---> power boot pin
GPA1  			 ---> 					  ---> lock disable detection

1.NB downlink 
  O  0			//unlock rope
  
  
  
2.NB UpLink
  
  L  0			//unlocked rope
  L  1 			//locked rope
  W	 1 			//drop warnning
  W  2			//not working warnning
  W  3  		//unlock in the air warnning
  
  R	 1 			//request unlock rope