/*******************************
 * Name: Abdul Hadi Khan
 * Student ID#:1000785873
 * Lab Day: Friday 15th of May
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 7 (ABET): Building a PIC18F4520 Standalone Alarm System with EUSART Communication
 ********************************/
#include <delays.h>
#include <stdio.h>
#include <EEP.h>
#include <math.h>


// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

#include <p18F4520.h>

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)



#define RED PORTBbits.RB2           //defined LEDs by their respective colors
#define GREEN PORTBbits.RB3         //
#define BLUE PORTBbits.RB4          //
#define YELLOW PORTBbits.RB5        //
#define PIR PORTBbits.RB0           //defined PIR sensor as PIR
#define C4 PORTDbits.RD4            //defined keypad inputs as C and their respective pin numbers
#define C5 PORTDbits.RD5
#define C6 PORTDbits.RD6
#define C7 PORTDbits.RD7            //
#define R0 PORTDbits.RD0            //defined keypad outputs as C and their respective pin numbers
#define R1 PORTDbits.RD1
#define R2 PORTDbits.RD2
#define R3 PORTDbits.RD3            //
#define HI 1
#define LO 0

void key1();                        //function for the keypad to manage blue light and delay
void initial();                     //function to initialise everything in the system
void motion();                      //sets the configuration for PIR sensor
void heat();                        //sets the configuration for the TMP36 sensor
void menu();                        //defines the menu and the lets the user select options
void authentication();              //authenticates whether the user has access to the system
void password();                    //confirms old password and lets the user select a new password
void chk_pass();                    //cheks if the password entered by the user is correct or not
void sys_info();                    //prints out the current state the system is in
void My_ISR_High(void);
void My_ISR_Low(void);
void keypad();                     //defines what values will appear based on the keypad input
int pass_true=0;
int i;
int threshold=75;
int thres_temp=0;
int try=0;
int tempT;
int tempM;
int form_input;
unsigned char form=8;
int low;
int high;
float voltage;
char buf='0';
unsigned int val;
unsigned int t;
char pass[4]="1111";
char key='0';
int check=1;
char inp_buffer[2];
char input[4]="1111";
unsigned char memory=0,p1=1,p2=2,p3=3,p4=4;
unsigned char pir_sense=7;
unsigned char tmp36=9;
unsigned char thres=5;
unsigned char mot=6;
char menu_sel;
int test;


#pragma code My_Hi_Priority_Int = 0x0008
void My_Hi_Priority_Int(void) {
    _asm
        //added a GOTO instruction to signal the high interrupt vector
        GOTO My_ISR_High
    _endasm
}
//Set the low priority interrupt and made sure that it has different location
#pragma code My_Lo_Priority_Int = 0x00018
void My_Lo_Priority_Int(void) {
    _asm
        //GOTO instruction to signal low interrupt vector
        GOTO My_ISR_Low
    _endasm
}
#pragma interrupt My_ISR_High
void My_ISR_High(void)
{
    if(INTCON3bits.INT1IF==1){          //interrupt if triggered means that PIR senser went off

        motion();                       //goes into motion function and asks the user for their password
        INTCON3bits.INT1IF=0;           //resets the interrupt so that it can be triggered later
    }
    //authentication();

}

//Setting up the high priority interrupt vector
#pragma interrupt My_ISR_Low
void My_ISR_Low(void)
{

   //Delay10KTCYx(250);
   if(INTCONbits.TMR0IF=1){             //interrrupt for the timer that is set to trigger after half a second
       TMR0H=0XB3;                      //set timer high buffer
       TMR0L=0XB4;                      //set timer low buffer
       ADCON0bits.GO= 1;                //turns on the a/d convertor
       INTCONbits.TMR0IF=0;             //resets the timer interrupt flag

   }
   if(PIR1bits.ADIF==1) {               //if the a/d conversion flag has been set convert the analog value to digital
        low = ADRESL;                   //Takes the low bits and saves them to variable 'low'
        high = ADRESH;                  //Takes the high bits from the input and stores them into variable 'high'
        high = high << 8;               //Shifts the high bits to the right by 8 places so that the entire input can be read
        voltage = low + high;           //Add both the high and the low bits to generate an input voltage value
        voltage = voltage * 5.0;        //get the value in decimals
        voltage = (voltage / 1023.0);   //account for the prescaler
        voltage = voltage-0.5;          //
        voltage = voltage*100;          //
        voltage = ((voltage*9)/5)+32;   //convertion to fareinhiet
        thres_temp = (int)(voltage);    //rounds down the value to an integer
        if(tempT==1){                   //if temperature value is in the eeprom
            if(thres_temp>threshold){   //checks if the sensor value is greater than the threshold that was set
                heat();                 //if so calls the heat function that asks the user for a password to disable the alarm
            }
        }
        PIR1bits.ADIF=0;                //reset a/d converter flag to use in the future
    }

}

void initial() {                        //initialise all that is neede for this system to function
    INTCONbits.GIE=1;                   //turn the global interrupt on
    INTCONbits.PEIE=1;                  //enable all peropheral interrupts
    INTCONbits.TMR0IE=1;                //enable timer interrupt
    INTCON2bits.TMR0IP=0;               //set the priority for the timer interrupt low since PIR sensor has high priority
    INTCON3bits.INT1E=1;
    INTCON3bits.INT1IP=1;
    INTCON2bits.INTEDG1=1;
    INTCON2bits.INTEDG0=1;
    TRISBbits.RB0 = 0;                  //set buzzer so pin has to be an output
    TRISBbits.RB1 = 1;                  //PIR sensor as input
    TRISAbits.RA0 = 1;                  //temperature sensor as input
    RCONbits.IPEN=1;                    //we enable two level of interrupts

    TRISBbits.RB2 = 0;                  //red led as output
    RED = LO;                           //turn red led off
    TRISBbits.RB3 = 0;                  //green set LEDs as outputs
    GREEN = HI;                         //green will on when system is on
    TRISBbits.RB4 = 0;                  //blue led as output
    BLUE=0;                             //turn it off
    TRISBbits.RB5 = 0;                  //yellow as output
    YELLOW=LO;                          //turn it off

    //keypad inputs
    TRISDbits.RD4 = 1; //in pin4
    TRISDbits.RD5 = 1; //in pin5
    TRISDbits.RD6 = 1; //in pin6
    TRISDbits.RD7 = 1; //in pin7
    //keypad outputs
    TRISDbits.RD0 = 0; //out pin1
    TRISDbits.RD1 = 0; //out pin2
    TRISDbits.RD2 = 0; //out pin3
    TRISDbits.RD3 = 0; //out pin4
    TRISCbits.RC7=1;   //RX as input (flipped)
    TRISCbits.RC6=0;   //TX as output (flipped)
    ADCON0=0b00000001;
    ADCON1=0b00001110;
    ADCON2=0b10101110;
    Delay100TCYx(100);

    TRISBbits.RB0 = 1; //PIR sensor as input
    RCONbits.IPEN=1;   //we enable two level of interrupts
    PIE1bits.RCIE=0;   //interrupt pririty for RX as low
    PIE1bits.TXIE=0;   //interrupt priority for TX as low
    PIE1bits.ADIE=1;   //a/d converter interrupt enabled
    IPR1bits.RCIP=0;   //recieving interrupt is off
    IPR1bits.TXIP=0;   //transmittion interrupt is off
    IPR1bits.ADIP=0;

    TMR0H=0XB3;        //reset the timer buffers
    TMR0L=0XB4;        //
    T0CON=0b10000110;  //turns the timer on with prescaler set
    SPBRG = 32;        //baud rate for the system
    TXSTAbits.TX9 = 0;
    TXSTAbits.TXEN = 1;
    TXSTAbits.SYNC = 0;//asynchronous transmition
    TXSTAbits.CSRC=0;

    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;//continous receiving
    form_input=0;       //select keyboard
    INTCONbits.GIE=0;
}

#pragma code
void main()
{

    initial();                                  //initialise the system
    while (1) {
        if (Read_b_eep(memory) == 1) {          //read memory eeprom
            pass[0] = Read_b_eep(p1);           //get first character of the password and so on
            pass[1] = Read_b_eep(p2);           //
            pass[2] = Read_b_eep(p3);           //
            pass[3] = Read_b_eep(p4);           //
            printf("\n\rPassword exists");
            try=1;
            break;
        }
        if(try==0){
            password();                         //if password not found let user enter new password
        }

        if(try=1){
            chk_pass();                         //if there was already a password in the system, verify the user knows it
        }

        if(try==4){
            for(i=0;i<4;i++) {
                Write_b_eep(memory, 1);
                Write_b_eep(i+1, input[i]);
            }
            INTCONbits.GIE=1;
            break;
        }

   }
    if(Read_b_eep(form)==1 || Read_b_eep(form)==0){
        form_input=Read_b_eep(form);
    }
    authentication();                               //check if the user has rights to the current system
    menu();                                         //if the user has right enable the menu to him/her


} //end of void main()

//Be sure to have a blank line at the end of your program

void sys_info(){                                        //prints out the system state for the user to view
    printf("\n\r");
    if(form_input==1){
        printf("\n\rInput via keypad");
    }
    else{
        printf("\n\rInput via keyboard");
    }
    if(INTCON3bits.INT1E==1){                           //if the INT1 is set that means the pir sensor is enbled
        printf("\n\rMotion sensor: ENABLED");
    }
    else{
        printf("\n\rMotion sensor: DISABLED");          //else it is disabled
    }
    if(tempT==1){
        printf("\n\rTemperature sensor: ENABLED");      //if tempT in the memory is 1, that means that tmp36 senser is enabled
    }
    else{
        printf("\n\rTemperature sensor: DISABLED");     //else disabled
    }
    printf("\n\rTemperature threshold: %d",threshold);  //print out the current threshold set for the temperature senser
    printf("\n\rTemperature sensor reads: ");           //print out the current reading from the temp sensor
    Delay10KTCYx(100);
    printf("%d",thres_temp);
    printf("\n\r");
    return;
}


void authentication(){                              //authenticate if the current user has right to access the system
    while(1){
        printf("\n\rEnter password: ");
        try=0;
        for(i=0;i<4;i++){                           //lets the user enter 4 digit passcode
            if(form_input==0){                      //checks if the form of input is keyboard
                while(PIR1bits.RCIF==0);            //stay while the data is transmitted
                buf=RCREG;                          //once done save it in the buffer
            }
            else{
                key1();                            //checks if the form of input is keypad
                buf=key;                           //save the key stroke in the buffer
            }
            input[i]=buf;                          //save the value from the buffer to inout array
            TXREG='*';                             //display a * instead of the actual character for privacy
            while(PIR1bits.TXIF==0);               //wait while the data is transfered
            Delay100TCYx(50);
        }
        pass[0] = Read_b_eep(p1);                  //read the values from eeprom
        pass[1] = Read_b_eep(p2);                  //
        pass[2] = Read_b_eep(p3);
        pass[3] = Read_b_eep(p4);                  //
        try=0;
        for(i=0;i<4;i++){                          //for the input entered by the user, checks if it is the same in memory
            if(input[i]==pass[i]){
            try++;                                 //increment try after verifying the character is in memory
            }
        }
        if(try==4){                                //if the password is in the memory, save it again just in case
             Write_b_eep(memory, 1);
             Write_b_eep(p1, input[0]);
             Write_b_eep(p2, input[1]);
             Write_b_eep(p3, input[2]);
             Write_b_eep(p3, input[3]);
             printf("\n\rPassword correct");       //dispay that the passowrd was correct
             break;
        }
        else{
            printf("\n\rPassword incorrect");
        }
    }
    tempM=1;
    if(Read_b_eep(pir_sense)==0 ||Read_b_eep(pir_sense)==1 ){       //checks if the values for the alarms are stored in the memory
        INTCON3bits.INT1E=Read_b_eep(pir_sense);
    }
    tempT=1;
    if(Read_b_eep(tmp36)==0 || Read_b_eep(tmp36)==1){
        tempT=Read_b_eep(tmp36);
    }
    if(Read_b_eep(thres)!=0xFF){
        threshold=Read_b_eep(thres);
    }
    return;
}
//request user to set a new password in case of new system or user wanting to change passwords
void password(){
    printf("\n\rEnter new password: ");

    for(i=0;i<4;i++){
        while (PIR1bits.RCIF==0);
        input[i]=RCREG;
        TXREG='*';
        while(PIR1bits.TXIF==0);
        Delay100TCYx(50);
    }
    //for(i=0;i<4;i++) {
    //write the new passord to the memory
    Write_b_eep(memory, 1);
    Write_b_eep(p1, input[0]);
    Write_b_eep(p2, input[1]);
    Write_b_eep(p3, input[2]);
    Write_b_eep(p3, input[3]);
    //}
    printf("\n\rPassword set.");
    return;
}
//checks the password that was entered by the user
void chk_pass(){
    printf("\n\rEnter password: ");
    Delay100TCYx(50);
    Delay100TCYx(50);
    for(i=0;i<4;i++){
        while (PIR1bits.RCIF==0);
        input[i]=RCREG;
        TXREG='*';
        while(PIR1bits.TXIF==0);
        //Delay100TCYx(50);
    }
    try=0;
    for(i=0;i<4;i++){
        if(input[i]==pass[i]){
        try++;
        }
    }
    if(try!=4){
        printf("\n\rIncorrect password");
        Delay100TCYx(50);
        chk_pass();
        }

    if (try==4){
        printf("\n\rPassword correct");
    }
    return;
}
//heat function to perform actions if the temp reading is above the threshold
void heat(){
    try=0;
    i=0;
    INTCONbits.TMR0IE=0;
    YELLOW=HI;                  //turns the yellow led on
    PORTBbits.RB0=HI;           //turns the buzzer on
    printf("\n\rHeat detected");
    printf("\n\rEnter password:");  //ask the user to authenticate
    pass[0] = Read_b_eep(p1);
    pass[1] = Read_b_eep(p2);
    pass[2] = Read_b_eep(p3);
    pass[3] = Read_b_eep(p4);
    while(1){
        for(i=0;i<4;i++){       //takes input from the keypad
        if(form_input==1){
            key1();
        }
        else{                   //takes input from the keyboard
            while(PIR1bits.RCIF==0);
            buf=RCREG;
            key=buf;
        }
        TXREG='*';              //same as before
        while(PIR1bits.TXIF==0);
        //Delay100TCYx(50);
        input[i]=key;
        }

        if(i==4){
            for(i=0;i<4;i++){
                if(input[i]==pass[i]){
                    i++;
                }
                else{
                    i=0;
                    try=0;
                    printf("\n\rIncorrect code");
                    printf("\n\rEnter password:");
                    break;
                }
                break;
            }
        }
        if(i==4){
            break;
        }
    }
    printf("\n\rTemperature sensor disabled");//if the user authenticates successfully, disable the alarm
    YELLOW=LO;                                //turn the led off
    PORTBbits.RB0=LO;                         //shut the buzzer
    INTCONbits.TMR0IE=1;                      //reset all the interrupts
    INTCON3bits.INT1E=1;
    Write_b_eep(tmp36,0);                     //save in memory that the heat sensor is disabled
    return;
}
//handles in there was motion in front of the pir
void motion(){
    try=0;
    i=0;
    INTCONbits.TMR0IE=0;            //resets the timer
    RED=HI;                         //turns the red led on
    INTCON3bits.INT1IE=0;           //turn off the interrupt
    printf("\n\rMotion detected");
    printf("\n\rEnter password:");  //asks user for the password
    while(1){
        pass[0] = Read_b_eep(p1);   //same as before
        pass[1] = Read_b_eep(p2);
        pass[2] = Read_b_eep(p3);
        pass[3] = Read_b_eep(p4);
        if(form_input==1){          //checks for form of inout
            key1();
        }
        else{
            while(PIR1bits.RCIF==0);
            key=RCREG;
        }
        TXREG='*';
        while(PIR1bits.TXIF==0);
        Delay100TCYx(50);
        input[i]=key;
        i++;
        if(i==4){
            for(i=0;i<4;i++){
                if(input[i]==pass[i]){
                    i++;
                }
                else{
                    i=0;
                    try=0;
                    printf("\n\rIncorrect code");
                    printf("\n\rEnter password");
                    break;
                }
            }
        }
        if(i==4){
            break;
        }
    }
    printf("\n\rMotion sensor disabled");//if the passowrd is valid the sensor is disabled
    RED=LO;
    INTCONbits.TMR0IE=1;                 //interrupts are reset
    INTCON3bits.INT1E=1;
    //return;
}



void menu(){
    while(1){
    printf("\n\r");
    printf("\n\r####~MENU~####");
    printf("\n\r1:Change temperature threshold");
    printf("\n\r2:Change password");
    printf("\n\r3:Switch inputs");
    printf("\n\r4:Turn ON/OFF motion");
    printf("\n\r5:Turn ON/OFF temp sensor");
    printf("\n\r6:System info");
    printf("\n\r");
    if(form_input==1){                          //menu acces for keypad

        key1();
        BLUE=HI;
        TXREG=key;
        while(PIR1bits.TXIF==0);
        Delay100TCYx(250);
        menu_sel=key;

        if(menu_sel=='1'){                      //allow user to set a new threshold for the sensor
            printf("\n\rPresent threshold: %d",threshold);
            printf("\n\rEnter new threshold: ");
            Delay100TCYx(100);
            for(i=0;i<2;i++){
            inp_buffer[i]=key;
            TXREG=inp_buffer[i];
            while(PIR1bits.TXIF==0);
            Delay100TCYx(250);
            }
            inp_buffer[0]=(inp_buffer[0]*10)+inp_buffer[1];//calculates the threshold for the sensor
            threshold=(inp_buffer[0]-16);                  //saves in the variable that will be saved in the memory
            Write_b_eep(tempT, threshold);
            printf("\n\rNew value for temperature has been set as %d",threshold);
        }
        if(menu_sel=='2'){  //allows user to change passwords
            while(1){
            printf("\n\rEnter password: ");
            Delay100TCYx(100);
            for(i=0;i<4;i++){
                BLUE=HI;
                buf=RCREG;
                input[i]=buf;
                TXREG='*';
                while(PIR1bits.TXIF==0);
                Delay100TCYx(100);
                BLUE=LO;
            }
            pass[0] = Read_b_eep(p1);
            pass[1] = Read_b_eep(p2);
            pass[2] = Read_b_eep(p3);
            pass[3] = Read_b_eep(p4);
            try=0;
            for(i=0;i<4;i++){
                if(input[i]==pass[i]){
                try++;
                }
            }
            if(try!=4){
                printf("\n\rIncorrect password");

            }

            if (try==4){
                printf("\n\rPassword correct");
                printf("\n\rEnter new password: ");
                for(i=0;i<4;i++){
                    BLUE=HI;
                    while (PIR1bits.RCIF==0);
                    input[i]=RCREG;
                    TXREG='*';
                    while(PIR1bits.TXIF==0);
                    Delay100TCYx(50);
                    BLUE=LO;
                }
                BLUE=HI;
                //for(i=0;i<4;i++) {
                Write_b_eep(memory, 1);
                Write_b_eep(p1, input[0]);
                Write_b_eep(p2, input[1]);
                Write_b_eep(p3, input[2]);
                Write_b_eep(p3, input[3]);
                Delay100TCYx(50);
                //}
                printf("\n\rPassword set.");
                break;
            }
        }
        }

        if(menu_sel=='3'){                      //allows user to change the form of input
            if(form_input==0){                  //if the default is keyboard set to keypad and vice versa
                form_input=1;
                Write_b_eep(form, 1);
                printf("\n\rKeypad selected.");
            }
            else{
                form_input=0;
                Write_b_eep(form, 0);
                printf("\n\rKeyboard selected.");
            }
        }

        if(menu_sel=='4'){                              //disable or enable the motion sensor according to its current state
            if(INTCON3bits.INT1E==1){
                INTCON3bits.INT1E=0;
                Write_b_eep(pir_sense, 0);
                printf("\n\rMotion sensor: Disabled");
            }
            else{
                INTCON3bits.INT1E=1;
                Write_b_eep(pir_sense, 0);
                printf("\n\rMotion sensor: Enabled");
            }
        }
        if(menu_sel=='5'){                              //enable or disable temp sensor based on it's state as in the memory
            if(tempT==1){
                tempT=0;
                Write_b_eep(tmp36, 0);
                printf("\n\rTemperature sensor: Disabled");
            }
            else{
                tempT=1;
                Write_b_eep(tmp36, 1);
                printf("\n\rTemperature sensor: Enabled");
            }
        }

        if(menu_sel=='6'){                                  //display the status the system is in
            sys_info();
        }
        Delay100TCYx(250);
        BLUE=LO;
    }
    else{                                                   //do the similar for keyboard entry
        BLUE=LO;
        while(PIR1bits.RCIF==0);
        menu_sel=RCREG;


        if(menu_sel=='1'){
            printf("\n\rPresent threshold: %d",threshold);
            printf("\n\rEnter new threshold: ");
            for(i=0;i<2;i++){
            while(PIR1bits.RCIF==0);
            inp_buffer[i]=RCREG;
            TXREG=inp_buffer[i];
            while(PIR1bits.TXIF==0);
            Delay100TCYx(250);
            }
            inp_buffer[0]=(inp_buffer[0]*10)+inp_buffer[1];
            threshold=(inp_buffer[0]-16);
            Write_b_eep(tempT, threshold);
            printf("\n\rNew value for temperature has been set as %d",threshold);
        }
        if(menu_sel=='2'){
            while(1){
            printf("\n\rEnter password: ");
            Delay100TCYx(50);
            //Delay100TCYx(50);
            for(i=0;i<4;i++){
                while (PIR1bits.RCIF==0);
                buf=RCREG;
                input[i]=buf;
                TXREG='*';
                while(PIR1bits.TXIF==0);
                Delay100TCYx(10);
            }
            pass[0] = Read_b_eep(p1);
            pass[1] = Read_b_eep(p2);
            pass[2] = Read_b_eep(p3);
            pass[3] = Read_b_eep(p4);
            try=0;
            for(i=0;i<4;i++){
                if(input[i]==pass[i]){
                try++;
                }
            }
            if(try!=4){
                printf("\n\rIncorrect password");
                break;
            }

            if (try==4){
                printf("\n\rPassword correct");
                printf("\n\rEnter new password: ");
                for(i=0;i<4;i++){
                    while (PIR1bits.RCIF==0);
                    input[i]=RCREG;
                    TXREG='*';
                    while(PIR1bits.TXIF==0);
                    Delay100TCYx(50);
                }
                //for(i=0;i<4;i++) {
                Write_b_eep(memory, 1);
                Write_b_eep(p1, input[0]);
                Write_b_eep(p2, input[1]);
                Write_b_eep(p3, input[2]);
                Write_b_eep(p3, input[3]);
                Delay100TCYx(50);
                //}
                printf("\n\rPassword set.");
            }

        }
            break;
        }

        if(menu_sel=='3'){
            if(form_input==0){
                form_input=1;
                Write_b_eep(form, 1);
                printf("\n\rKeypad selected.");
            }
            else{
                form_input=0;
                Write_b_eep(form, 0);
                printf("\n\rKeyboard selected.");
            }
        }

        if(menu_sel=='4'){
            if(INTCON3bits.INT1E==1){
                INTCON3bits.INT1E=0;
                Write_b_eep(pir_sense, 0);
                printf("\n\rMotion sensor: Disabled");
            }
            else{
                INTCON3bits.INT1E=1;
                Write_b_eep(pir_sense, 0);
                printf("\n\rMotion sensor: Enabled");
            }
        }
        if(menu_sel=='5'){
            if(tempT==1){
                tempT=0;
                Write_b_eep(tmp36, 0);
                printf("\n\rTemperature sensor: Disabled");
            }
            else{
                tempT=1;
                Write_b_eep(tmp36, 1);
                printf("\n\rTemperature sensor: Enabled");
            }
        }

        if(menu_sel=='6'){
            sys_info();
        }
    }

    }
    return;
}


void keypad(){
    key=' ';
    R0=HI;
    while(R0==HI){              //set the row0 to keypad as high
        if(C4==HI){             //if column4 is high, means that '1' was pressed
            key='1';
            test=1;
            return;             //return the value
        }
        else if(C5==HI){        //with Row0 still high, and col5 is high
            key='2';            //return 2
            test=1;
            return;
        }
        else if(C6==HI){        //row0 high, col6 high
            key='3';            //return 3
            test=1;
            return;
        }
        else if(C7==HI){        //row0 high, col7 high
            key='A';            //return A
            test=1;
            return;
        }
        else{
            R0=LO;              //turn row0 low
            R1=HI;              //turn row1 high
        }
    }
    while(R1==HI){              //row1 high,
        if(C4==HI){             //col4 high
            key='4';            //return4
            test=1;
            return;
        }
        else if(C5==HI){        //this is basically how we access the keys from the key pad
            key='5';
            test=1;
            return;
        }
        else if(C6==HI){
            key='6';
            test=1;
            return;
        }
        else if(C7==HI){
            key='B';
            test=1;
            return;
        }
        else{
            R1=LO;
            R2=HI;
        }
    }
    while(R2==HI){
        if(C4==HI){
            key='7';
            test=1;
            return;
        }
        else if(C5==HI){
            key='8';
            test=1;
            return;
        }
        else if(C6==HI){
            key='9';
            test=1;
            return;
        }
        else if(C7==HI){
            key='C';
            test=1;
            return;
        }
        else{
            R2=LO;
            R3=HI;
        }
    }
    while(R3==HI){
        if(C4==HI){
            key='*';
            test=1;
            return;
        }
        else if(C5==HI){
            key='0';
            test=1;
            return;
        }
        else if(C6==HI){
            key='#';
            test=1;
            return;
        }
        else if(C7==HI){
            key='D';
            test=1;
            return;
        }
        else{
            R3=LO;
            break;
        }
    }
    Delay10KTCYx(100);          //create a delay after a key is pressed so that we are sure data was transmitted

}


void key1(){
    test=0;
    BLUE=HI;
    while(test!=1){
        keypad();
        Delay10KTCYx(100);
    }
    BLUE=LO;
}

