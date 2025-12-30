#include "mbed.h"

I2C i2c(P0_10,P0_11);     // sda, scl
Serial uart0(P0_4,P0_0,9600);    //UART0 TX, RX, for debug
DigitalIn sw1(P0_17);
DigitalIn sw2(P0_13);
DigitalIn sw3(P0_16);
DigitalIn sw4(P0_15);

#define sw_wait 0.2 //SW wait time

//lcd
const uint8_t contrast=40;  //lcd contrast
const uint8_t lcd_addr=0x7C;   //lcd i2c addr 0x7C
void lcd_init(uint8_t addr, uint8_t contrast);     //lcd init func
void char_disp(uint8_t addr, uint8_t position, char data);
void val_disp(uint8_t addr, uint8_t position, uint8_t digit,uint16_t val);

//AP33772S&INA233
uint8_t ina_addr=0x80;   //ina232/236 i2c addr 0x40<<1(A). Or 0x48<<1(B).
const uint8_t ap_addr=0x52<<1;     //AP33772 address
const uint8_t PDREQ=0x31;
const uint8_t SRCPDO=0x20;
const uint8_t VOLTAGE=0x11;
const uint8_t CURRENT=0x12;
char s_buf[3];  //send buffer
char r_buf[26]; //receive buffer
uint8_t num, sel_pdo=1;    //number of all pdos, selected pdo
uint16_t pps_vol, max_cur;   //PPS voltage, PDO max current
uint8_t table[13][4];   //PDO table

//INA232/236
void meas();
uint16_t vbus, cur;  //mV unit, mA unit
float vbus_f, v_shunt, i_f;      //mV, uV, mA unit
#define shunt_R 0.005
#define v_ofs 0
#define v_gain 1
#define c_ofs 0
#define c_gain 1
#define c_ofs_com 5 //AP33772S Iq. Typ. 3.3mA, Max. 6mA.

int main(){
    //SW init.
    sw1.mode(PullUp);
    sw2.mode(PullUp);
    sw3.mode(PullUp);
    sw4.mode(PullUp);

    //LCD init.
    i2c.frequency(100000);  //I2C clk 100kHz
    wait(0.1);  //wait for LCD power on
    lcd_init(lcd_addr, contrast);
    wait(0.2);
    char_disp(lcd_addr,0x40+2,'.');
    char_disp(lcd_addr,0x40+6,'V');
    char_disp(lcd_addr,0x40+9,'.');
    char_disp(lcd_addr,0x40+13,'A');
    
    //INA init. INA addr check
    s_buf[0]=0x3e;  //ID reg. addr
    i2c.write(ina_addr,s_buf,1);
    i2c.read(ina_addr|1,r_buf,2);
    if(r_buf[0]==0x54) uart0.printf("INA Addr is 0x40\n\r");
    else{
        ina_addr=ina_addr+0x10;
        i2c.write(ina_addr,s_buf,1);
        i2c.read(ina_addr|1,r_buf,2);
        if(r_buf[0]==0x54) uart0.printf("INA Addr is 0x48\n\r");
        else{
            ina_addr=0;
            uart0.printf("NO INA\n\r");
        }
    }
    
    //INA config.
    s_buf[0]=0;
    s_buf[1]=(0b10<<5)+(0b0<<4)+(0b011<<1)+0b1; //FS, AVG, VBUSCT
    s_buf[2]=(0b00<<6)+(0b100<<3)+0b111;        //VBUSCT, VSHCT, MODE
    i2c.write(ina_addr,s_buf,3);
    
    //pdo check
    wait(0.5);  //wait for AP33772S power on
    s_buf[0]=SRCPDO;
    i2c.write(ap_addr,s_buf,1);
    i2c.read(ap_addr|1,r_buf,26);
    if((r_buf[1]>>6)==0){   //PDO is not detected.
        uart0.printf("PDO is not detected.\n\r");
        char_disp(lcd_addr,0,'N');
        char_disp(lcd_addr,1,'O');
        char_disp(lcd_addr,2,'T');
        char_disp(lcd_addr,3,' ');
        char_disp(lcd_addr,4,'P');
        char_disp(lcd_addr,5,'D');
        while (true){   //NO PD mode.
            meas();
            wait(0.2);
        }
    }else{  //PDo is detected.
        for (uint8_t i=1;i<=7;++i){ //SPR PDO
            if((r_buf[2*i-1]>>6)==0) break;
            uart0.printf("PDO%d: EN=%x, CUR=%x, PC/VM=%x, VOL=%dmV\n\r",i, r_buf[2*i-1]>>6,(r_buf[2*i-1]>>2)&0b1111,(r_buf[2*i-1])&0b11,r_buf[2*i-2]*100);
            num++;
            table[num][0]=(uint8_t)(r_buf[2*i-2]/10);   //voltage
            table[num][1]=i;    //index
            table[num][2]=(r_buf[2*i-1]>>2)&0b1111; //current
            table[num][3]=r_buf[2*i-1]>>6;  //type. 2=Fixed, 3=APDO
        }
        for (uint8_t i=8;i<=13;++i){ //EPR PDO
            if((r_buf[2*i-1]>>6)==0)break;
            uart0.printf("PDO%d: EN=%x, CUR=%x, PC/VM=%x, VOL=%dmV\n\r",i, r_buf[2*i-1]>>6,(r_buf[2*i-1]>>2)&0b1111,(r_buf[2*i-1])&0b11,r_buf[2*i-2]*200);
            num++;
            table[num][0]=(uint8_t)(r_buf[2*i-2]/5);
            table[num][1]=i;
            table[num][2]=(r_buf[2*i-1]>>2)&0b1111;
            table[num][3]=r_buf[2*i-1]>>6;  //type. 2=Fixed, 3=APDO
        }
    }

    while (true){
        val_disp(lcd_addr,0,2,table[sel_pdo][0]);
        char_disp(lcd_addr,2,'V');
        char_disp(lcd_addr,3,' ');
        max_cur=table[sel_pdo][2]*250+1000; //mA unit
        if(max_cur==4750)max_cur=5000;
        val_disp(lcd_addr,4,1,max_cur/1000);
        char_disp(lcd_addr,5,'.');
        val_disp(lcd_addr,6,1,(max_cur/100)%10);
        char_disp(lcd_addr,7,'A');
        char_disp(lcd_addr,8,' ');
        val_disp(lcd_addr,9,1,sel_pdo);
        char_disp(lcd_addr,10,'/');
        val_disp(lcd_addr,11,1,num);
        char_disp(lcd_addr,12,' ');
        meas();

        if(sw4==0)sel_pdo=sel_pdo+1;    //pdo up
        if(sw3==0)sel_pdo=sel_pdo-1;    //pdo down
        if(sel_pdo>num)sel_pdo=1;
        if(sel_pdo<1)sel_pdo=num;
        val_disp(lcd_addr,9,1,sel_pdo);
        val_disp(lcd_addr,0,2,table[sel_pdo][0]);
        if(table[sel_pdo][3]==3){
            char_disp(lcd_addr,13,'P');
            char_disp(lcd_addr,14,'P');
            char_disp(lcd_addr,15,'S');
        }else{
            char_disp(lcd_addr,13,' ');
            char_disp(lcd_addr,14,' ');
            char_disp(lcd_addr,15,' ');
        }
        wait(sw_wait);

        if(sw2==0){
            if(table[sel_pdo][3]==3){  //PPS variable voltage request
                s_buf[0]=PDREQ;
                s_buf[1]=0x32;//5V req first
                s_buf[2]=0x0+((table[sel_pdo][1])<<4)+(table[sel_pdo][2]); //APDO index, max current
                i2c.write(ap_addr,s_buf,3);
                pps_vol=5000;   //mV unit. default
                val_disp(lcd_addr,0,2,pps_vol/1000);
                char_disp(lcd_addr,2,'.');
                val_disp(lcd_addr,3,1,(pps_vol%1000)/100);
                char_disp(lcd_addr,4,'V');
                char_disp(lcd_addr,5,' ');
                char_disp(lcd_addr,6,'P');
                char_disp(lcd_addr,7,'P');
                char_disp(lcd_addr,8,'S');
                char_disp(lcd_addr,9,' ');
                char_disp(lcd_addr,10,'M');
                char_disp(lcd_addr,11,'x');
                max_cur=table[sel_pdo][2]*250+1000; //mA unit
                if(max_cur==4750)max_cur=5000;
                val_disp(lcd_addr,12,1,max_cur/1000);
                char_disp(lcd_addr,13,'.');
                val_disp(lcd_addr,14,1,(max_cur/100)%10);
                char_disp(lcd_addr,15,'A');
                wait(sw_wait);
                while (true){
                    if((sw3==0)&&(sw4==0)) break;       //break out of the PPS loop
                    if(sw2==0)pps_vol=pps_vol+1000;    //1V up
                    if(sw1==0)pps_vol=pps_vol-1000;    //1V down
                    if(sw4==0)pps_vol=pps_vol+100;    //0.1V up
                    if(sw3==0)pps_vol=pps_vol-100;    //1V up
                    if(pps_vol>=(table[sel_pdo][0]*1000))pps_vol=(table[sel_pdo][0]*1000);
                    if(pps_vol<=5000)pps_vol=5000;
                    s_buf[0]=PDREQ;
                    s_buf[1]=(char)(pps_vol/100);  //PPS Voltage req
                    s_buf[2]=0x0+((table[sel_pdo][1])<<4)+(table[sel_pdo][2]); //APDO index, max current
                    i2c.write(ap_addr,s_buf,3);
                    val_disp(lcd_addr,0,2,pps_vol/1000);
                    val_disp(lcd_addr,3,1,(pps_vol%1000)/100);
                    meas();
                    wait(sw_wait);
                }
            }else{  //fixed voltage request
                s_buf[0]=PDREQ;
                s_buf[1]=0x32;//no mean in fixed pdo.
                s_buf[2]=0x0+((table[sel_pdo][1])<<4)+(table[sel_pdo][2]); //fixed PDO index, max current
                i2c.write(ap_addr,s_buf,3);
            }
        }
    }    
}

//INA meas&LCD disp
void meas(){
    if(ina_addr==0){  //Use AP33772S internal meas.
        s_buf[0]=VOLTAGE;
        i2c.write(ap_addr,s_buf,1);
        i2c.read(ap_addr|1,r_buf,2);
        vbus=(uint16_t)(((r_buf[1]<<8)+r_buf[0])*80); //mV unit
        s_buf[0]=CURRENT;
        i2c.write(ap_addr,s_buf,1);
        i2c.read(ap_addr|1,r_buf,1);
        cur=(uint16_t)(r_buf[0]*24); //mA unit
    }else{  //Use INA
        //read vbus
        s_buf[0]=0x2;
        i2c.write(ina_addr,s_buf,1);
        i2c.read(ina_addr|1,r_buf,2);
        vbus_f=(((r_buf[0]<<8)+r_buf[1])*1.6-v_ofs)*v_gain;    //1LSB=1.6mV, mV unit
        vbus=(uint16_t)vbus_f;
        //read current
        s_buf[0]=0x1;
        i2c.write(ina_addr,s_buf,1);
        i2c.read(ina_addr|1,r_buf,2);
        v_shunt=((int16_t)((r_buf[0]<<8)+r_buf[1])*2.5-c_ofs)*c_gain;    //1LSB=2.5uV, uV unit
        i_f=v_shunt/shunt_R/1000;   //mA unit
        cur=(uint16_t)(abs(i_f)-c_ofs_com);
    }
    //disp
    val_disp(lcd_addr,0x40+0,2,vbus/1000);
    val_disp(lcd_addr,0x40+3,3,vbus%1000);
    val_disp(lcd_addr,0x40+8,1,cur/1000);
    val_disp(lcd_addr,0x40+10,3,cur%1000);
}

//disp char func
void char_disp(uint8_t addr, uint8_t position, char data){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;            //write command
    buf[1]=data;
    i2c.write(addr,buf,2);
}

//disp val func
void val_disp(uint8_t addr, uint8_t position, uint8_t digit, uint16_t val){
    char buf[2],data[4];
    uint8_t i;
    buf[0]=0x0;
    buf[1]=0x80+position;       //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    data[3]=0x30+val%10;        //1
    data[2]=0x30+(val/10)%10;   //10
    data[1]=0x30+(val/100)%10;  //100
    data[0]=0x30+(val/1000)%10; //1000
    buf[0]=0x40;                //write command
    for(i=0;i<digit;++i){
        if(i==0&&data[0]==0x30&&digit==4) buf[1]=0x20;
        else buf[1]=data[i+4-digit];
        i2c.write(addr,buf,2);
    }
}

//LCD init func
void lcd_init(uint8_t addr, uint8_t contrast){
    char lcd_data[2];
    lcd_data[0]=0x0;
    lcd_data[1]=0x38;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x39;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x14;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x70|(contrast&0b1111);
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x56|((contrast&0b00110000)>>4);
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x6C;
    i2c.write(addr,lcd_data,2);
    wait(0.2);
    lcd_data[1]=0x38;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x0C;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x01;
    i2c.write(addr,lcd_data,2);
}
