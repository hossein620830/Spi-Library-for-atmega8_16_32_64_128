

#include "spi.h"


void  spi_deinit(void)
{
      // change config pin
	    
		SPI_MISO_DDR &= ~(1<<SPI_MISO_PIN);//  configure miso pin to input
        SPI_SS_DDR &=~(1<< SPI_SS_PIN );//  configure ss pin to input
	 
     	 
	    SPI_MOSI_DDR &= ~(1<<SPI_MOSI_PIN);// set MOSI pin to input 
	    SPI_SCK_DDR  &= ~(1<<SPI_SCK_PIN);// set sck pin to input 
	// setting registers
	
	SPCR=0x00;
	SPSR&=  ~(1<<SPI2X);
}	


//@brife   for use out of interrupt  and no delay to communication
/* uint8_t    spi_communication_byte( uint8_t  data)
{
      	  
	  SPDR=data;// load data 
	      
	if (SPCR&(1<<SPIE))
	{
       while(spi_data_transfer_flg==0){};
	  
       spi_data_transfer_flg=0;
	   
	} 
	else
	{
	   while(!(SPSR&(1<<SPIF))){};// waite for data send
	   SPSR |= (1<<SPIF); // for clear spif   
	}
	  
	return   SPDR;
	
}//end of spi_master_communication_byte 	

*/

 uint8_t    spi_fast_communication_byte( uint8_t  data)
{
      	  
	  SPDR=data;// load data 
	      
	
	#if (SPI_VERSION_VOLUME==SPI_full_volume_version)
	
	    if ( SPCR&(1<< SPIE))
		{
	     	
             while(spi_data_transfer_flg==0){};
	  
             spi_data_transfer_flg=0;
		
	    }  		 
        else
	    {
             while(!(SPSR&(1<<SPIF))){};// waite for data send
	         SPSR |= (1<<SPIF); // for clear spif   
        }
	
    #elif (SPI_VERSION_VOLUME==SPI_low_volume_version ) 
 
	
	     #if (SPI_INTERRUPT ==  SPI_Enable )
		
	     	
             while(spi_data_transfer_flg==0){};
	  
             spi_data_transfer_flg=0;
		
	      		 
        #else
	
             while(!(SPSR&(1<<SPIF))){};// waite for data send
	         SPSR |= (1<<SPIF); // for clear spif   
        #endif
	#endif 
	  
	return   SPDR;
	
}//end of spi_master_communication_byte 	


uint8_t    _spi_fast_transmit_string_array_nbyte( uint8_t  *data, uint8_t     cmd_nbyte)
{ 
    #define     nbyte_msk           0x7F
	#define     cmd_msk             0x80
	#define     string_transmit     cmd_msk
	#define     data_transmit      0x00
	
	uint8_t  temp;
	register uint8_t    i=0;
	//while((i<(cmd_nbyte& nbyte_msk )) &&(( data[i]!='\0' )||( (cmd_nbyte&cmd_msk) != string_transmit )))
     while(i<(cmd_nbyte& nbyte_msk ))
     { 	  
	    SPDR=data[i];// load data 
	      
	    #if (SPI_VERSION_VOLUME==SPI_full_volume_version)
	
	        if ( SPCR&(1<< SPIE))
		    {
	     	
                 while(spi_data_transfer_flg==0){};
	  
                 spi_data_transfer_flg=0;
		
	        }  		 
            else
	        {
                while(!(SPSR&(1<<SPIF))){};// waite for data send
	            SPSR |= (1<<SPIF); // for clear spif   
            }
	
        #elif (SPI_VERSION_VOLUME==SPI_low_volume_version ) 
 
	
	        #if (SPI_INTERRUPT ==  SPI_Enable )
		
	     	
                 while(spi_data_transfer_flg==0){};
	  
                 spi_data_transfer_flg=0;
		
	      		 
            #else
	
                while(!(SPSR&(1<<SPIF))){};// waite for data send
	            SPSR |= (1<<SPIF); // for clear spif   
            #endif
	    #endif 
		
		if(( data[i]=='\0' )&&( (cmd_nbyte&cmd_msk) == string_transmit ))
		     {i++ ; break; }
			 
		i++;
		
	 } 
	  temp=SPDR; 
	 return i;  
	
}//end of  _spi_fast_transmit_string_nbyte 	


uint8_t    _spi_fast_recive_string_array_nbyte( uint8_t  *data, uint8_t     cmd_nbyte)
{
    #define     nbyte_msk           0x7F
	#define     cmd_msk             0x80
	#define     string_transmit     cmd_msk
	#define     data_transmit      0x00
	
	
	register uint8_t    i=0;
	while(i<(cmd_nbyte& nbyte_msk )) 
     { 	  
	    SPDR=0x00;// load data 
	      
	    
		
		#if (SPI_VERSION_VOLUME==SPI_full_volume_version)
	
	        if ( SPCR&(1<< SPIE))
		    {
	     	
                while(spi_data_transfer_flg==0){};
	  
                spi_data_transfer_flg=0;
		
	        }  		 
            else
	        {
                while(!(SPSR&(1<<SPIF))){};// waite for data send
	            SPSR |= (1<<SPIF); // for clear spif   
            }
	
        #elif (SPI_VERSION_VOLUME==SPI_low_volume_version ) 
 
	
	        #if (SPI_INTERRUPT ==  SPI_Enable )
		
	     	
                while(spi_data_transfer_flg==0){};
	  
                spi_data_transfer_flg=0;
		
	      		 
            #else
	
                while(!(SPSR&(1<<SPIF))){};// waite for data send
	            SPSR |= (1<<SPIF); // for clear spif   
            #endif
	    #endif 
						
		data[i] = SPDR;// load data
				
		if(( data[i]=='\0' )&&( (cmd_nbyte&cmd_msk) == string_transmit ))
		     {i++ ; break; }
	    i++;		 
		
	 } 
	   
	 return i;  
	
}//end of  _spi_fast_transmit_string_nbyte




#if (SPI_VERSION_VOLUME==SPI_low_volume_version)

void  __spi_init(uint8_t  cmd)
{ 
    		
	SPCR &= ~(1<<SPIE);// disable  spi interrupt
	SPCR &= ~(1<<SPE);// disable  spi
	// configure pins
	
	
	SPI_SS_PORT |= (1<< SPI_SS_PIN);//  set SS pin to high
	
	if  (cmd&(1<<7)) // if master is selected
	{
        
	    write_bits(  SPI_SS_DDR , SPI_SS_PIN , 1 , cmd) ;//  configure ss pin direction
	 
     	SPI_MISO_DDR &= ~(1<<SPI_MISO_PIN);//  configure miso pin to input 
	    SPI_MOSI_DDR |=(1<<SPI_MOSI_PIN);// set MOSI pin to output
	    SPI_SCK_DDR  |=(1<<SPI_SCK_PIN);// set sck pin to output
		
		
    }
    else // if slave is selected 
    {
        SPI_MISO_DDR |= (1<<SPI_MISO_PIN);//  configure miso pin to output
			
        SPI_SS_DDR &=~(1<< SPI_SS_PIN );//  configure ss pin to input
	 
     	 
	    SPI_MOSI_DDR &= ~(1<<SPI_MOSI_PIN);// set MOSI pin to input 
	    SPI_SCK_DDR  &= ~(1<<SPI_SCK_PIN);// set sck pin to input 
    }		
	
	
	
	 
	 // configure spi  registers
	  
	  
	 write_bits(  SPCR , SPIE , 1 , cmd>>1 ) ;// config interrupt
	 write_bits(  SPCR , DORD , 1 , SPI_DATA_ORDER ) ;// select msb or lsb first send
	 write_bits(  SPCR , MSTR , 1 , cmd>>7 ) ;// select master or slave
	 // config spi mode
	 write_bits(  SPCR , CPHA , 1 , cmd>>2 ) ;// set mode  cpha part
	 write_bits(  SPCR , CPOL , 1 , cmd>>3 ) ;// set mode  cpol part
	 
	 // config clock frequncy
	 
	 write_bits(  SPCR , SPR0  , 1 , cmd>>4 ) ;// set spr0 bit
	 write_bits(  SPCR , SPR1  , 1 , cmd>>5 ) ;// set spr1 bit
	 write_bits(  SPSR , SPI2X , 1 , cmd>>6 ) ;// set spi2x bit 
	 
	  
	 
	 if( ((cmd>>1)&0x01) == SPI_Enable)// if enable spi interrupt
	   {  __asm__ __volatile__ ("sei" ::);  }// enable global interrupt
	   
	 SPCR |= (1<<SPE);// Enable  spi
	 
	  if(  ((SPCR&(1<<MSTR)) ==(1<<MSTR)) && (  (SPI_SS_DDR&(1<< SPI_SS_PIN))==(1<< SPI_SS_PIN))  )
        SPI_SS_PORT &= ~(1<< SPI_SS_PIN);//  set SS pin to low enable spi     
	  
		 
  }// end of  __spi_init



#elif (SPI_VERSION_VOLUME==SPI_full_volume_version)


void     spi_active_ss_pin(spi_enable_disable_typedef    status)
{

      if (status == spi_enable)
	  { 
	     if(  ((SPCR&(1<<MSTR)) ==(1<<MSTR)) && (  (SPI_SS_DDR&(1<< SPI_SS_PIN))==(1<< SPI_SS_PIN))  )
                   SPI_SS_PORT &= ~(1<< SPI_SS_PIN);//  set SS pin to low enable spi 
      }
      else
	  {
           SPI_SS_PORT |= (1<< SPI_SS_PIN);//  set SS pin to h high disable spi slave 
	  } 	   
}





void     spi_enable_disable_device(spi_enable_disable_typedef    status)
{
     write_bits(  SPCR , SPE , 1 , status) ;//

}

void spi_config_interrupt(spi_enable_disable_typedef    status)
{
   // write_bits(  SPCR , SPIE , 1 ,  status ) ;// config interrupt 
	if(status == spi_enable )
	{
	   SPCR |= (1<<SPIE);
	        __asm__ __volatile__ ("sei" ::);
    }
    else
       SPCR &= ~(1<<SPIE);			
}


void spi_config_dataorder(spi_dataorder_typedef    order)
{
   write_bits(  SPCR , DORD , 1 , order) ;// select msb or lsb first send

}


void spi_config_mode(spi_mode_typedef   mode)
{
     write_bits(  SPCR , CPHA , 1 , mode ) ;// set mode  cpha part
	 write_bits(  SPCR , CPOL , 1 , mode>>1 ) ;// set mode  cpol part 
}	


void  spi_config_clock(spi_clock_typedef   clock)
{
     write_bits(  SPCR , SPR0  , 1 , clock ) ;// set spr0 bit
	 write_bits(  SPCR , SPR1  , 1 , clock>>1 ) ;// set spr1 bit
	 write_bits(  SPSR , SPI2X , 1 , clock>>2 ) ;// set spi2x bit 
}
uint8_t    Spi_clear_Transfer_flag_and_read(void)
{
     SPSR |= (1<<SPIF); // for clear spif
	 return SPDR; // read data and clear spif

}


void      Spi_clear_Transfer_flag_and_write(uint8_t   data)
{
  
     SPSR |= (1<<SPIF); // for clear spif
	 SPDR =  data; // read data and clear spif

}


void       Spi_clear_write_colision_flag_and_write(uint8_t   data)
{
  
     SPSR |= (1<<WCOL); // for clear spif
	 SPDR =  data; // read data and clear spif

}


//@brife : use this after  spi_config_master_slave Function
void     spi_ss_direct( spi_ss_direct_typedef   direct)
{
      
	  //SPI_SS_PORT |=(1<< SPI_SS_PIN);//  set SS pin to high
      spi_active_ss_pin(spi_disable);
      /*
	  if ( direct==SPI_Ss_Input)
	  {
	    
		SPI_MISO_DDR |= (1<<SPI_MISO_PIN);//  configure miso pin to output
	  } 
	  else
	  {
	    SPI_MISO_DDR &= ~(1<<SPI_MISO_PIN);//  configure miso pin to input
	  }
	   */
	
	write_bits(  SPI_SS_DDR , SPI_SS_PIN , 1 ,  direct ) ;// config direction

}




void spi_config_master_slave(spi_master_slave_typedef    master_slave , spi_ss_direct_typedef   ss_direct)
{
    //SPI_SS_PORT |=(1<< SPI_SS_PIN);//  set SS pin to high
	spi_active_ss_pin(spi_disable);//  set SS pin to high
	if  (master_slave == spi_master_mode) // if master is selected
	{
        
	 
	   spi_ss_direct(  ss_direct) ;//  configure ss pin direction
	 
     	SPI_MISO_DDR &= ~(1<<SPI_MISO_PIN);//  configure miso pin to input 
	    SPI_MOSI_DDR |=(1<<SPI_MOSI_PIN);// set MOSI pin to output
	    SPI_SCK_DDR  |=(1<<SPI_SCK_PIN);// set sck pin to output
    }
    else // if slave is selected 
    {   
	    
        SPI_MISO_DDR |= (1<<SPI_MISO_PIN);//  configure miso pin to output
        spi_ss_direct( spi_ss_input  );//  configure ss pin to input
	 
     	 
	    SPI_MOSI_DDR &= ~(1<<SPI_MOSI_PIN);// set MOSI pin to input 
	    SPI_SCK_DDR  &= ~(1<<SPI_SCK_PIN);// set sck pin to input 
    }	
	
	write_bits(  SPCR , MSTR , 1 ,     master_slave ) ;// select master or slave  
   // if(  ((SPCR&(1<<MSTR)) ==(1<<MSTR)) && (  (SPI_SS_DDR&(1<< SPI_SS_PIN))==(1<< SPI_SS_PIN))  )
   //     SPI_SS_PORT &= ~(1<< SPI_SS_PIN);//  set SS pin to low enable spi   
} 

void  __spi_init(uint8_t  cmd)
{ 
    		
	spi_config_interrupt(spi_disable );// disable  spi interrupt
	spi_enable_disable_device(spi_disable );//disable  spi  Device
	// configure pins
		
    spi_config_master_slave( (spi_master_slave_typedef)((cmd>>7)&0x01)  , (spi_ss_direct_typedef)(cmd&0x01) );
	
	 
	 // configure spi  registers
	  
	  
	 
	 spi_config_dataorder( SPI_DATA_ORDER ) ;// select msb or lsb first send
	 // config spi mode
	 spi_config_mode((spi_mode_typedef )((cmd>>2)&0x03) ) ;// set mode   part
	 
	 
	 // config clock frequncy
	 
	 spi_config_clock((spi_clock_typedef)((cmd>>4)&0x07) ) ;// set spr0 bit
	 spi_config_interrupt((spi_enable_disable_typedef) ((cmd>>1)&0x01) ) ;// config interrupt
	   
	 spi_enable_disable_device(spi_enable );// Enable  spi device
	 
	  //if(  ((SPCR&(1<<MSTR)) ==(1<<MSTR)) && (  (SPI_SS_DDR&(1<< SPI_SS_PIN))==(1<< SPI_SS_PIN))  )
       // SPI_SS_PORT &= ~(1<< SPI_SS_PIN);//  set SS pin to low enable spi     
	  spi_active_ss_pin(spi_enable);//  set SS pin to low
		 
  }// end of  __spi_init







#endif 



#if (SPI_INTERRUPT==SPI_Enable)

   ISR(SPI_STC_vect)
   {
      spi_data_transfer_flg=1;
	  

   }
#endif
