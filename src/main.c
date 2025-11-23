//SERIAL
#include "pragmas.h"

// ********** MACRO DEFINITIONS **********

//ascii values of the characters used in the commands
#define G 0x47
#define O 0x4f
#define E 0x45
#define N 0x4e
#define D 0x44
#define P 0x50
#define R 0x52
#define K 0x4b
#define X 0x58
#define T 0x54
#define S 0x53
#define U 0x55
#define B 0x42
#define M 0x4d
#define C 0x43
#define F 0x46
#define S_0 0x30
#define S_1 0x31
#define S_2 0x32
#define S_3 0x33
#define S_4 0x34
#define S_5 0x35
#define S_6 0x36
#define S_7 0x37
#define S_8 0x38
#define S_9 0x39

#define MSG_QUEUE_SIZE_X 128

// ********* USER DEFINED TYPES ************


typedef struct out_msg {
    int licence_num;  
    char level;
    int park_spot;
    char msg_type;
    int total_fee;
} out_msg;

// ******** FUNCTION DECLERATIONS ***********

void park_the_car(int carToPark); 
void execute_command(char message, int plateNumber, int slotNumber);
void licence_to_int(uint8_t a, uint8_t b, uint8_t c);
void park_spot_to_int(uint8_t a, uint8_t b);
void push_car_queue(int licence_num);
int pop_car_queue();
void push_msg_queue(out_msg msg);
out_msg pop_msg_queue();
int calculateFee(int entranceTime);
int int_to_char(int val, int divider);
void reserve_the_lot(int carToReserve, int slotNumber);
void exit(int carToExit);
int level_to_index(char level_char);

// ******** GLOBAL VARIABLES *********

int adc_level; //ADC value (potentiometer)
int adc_counter = 10; //counts since last adc check
int check_adc_flag = 1; //flag to check adc
int waiting = 1;
int finished = 0;
int licence_number = 0; //last car's license number
int park_spot = 0; //last car's park spot
int general_counter = 0;

int message_counter = 0;  //counts to 20 for 100ms message sending
int message_num = 0;
int out_valid = 0;

int display_digits[4] = {0x3f,0x3f,0x3f,0x3f}; //current 7-segment digit values
int update_display = 0; //flag to update display
int display_total_money = 1; // 1:show money, 0:show empty slots
int digit = 0; //active 7-segment digit
int total_money = 0;
uint8_t prev_rb4 = 0x10;
int empty_spaces[4] = {10,10,10,10}; //for each level
int display_level; //level to show empty lot count
int read_adc_value = 0; //flag for adc ready

char* curr_out;

typedef struct ParkingLot {
    int plateNumber;
    int isReserved;
    int entranceTime;
} ParkingLot;

typedef struct ReserveInfo { 
    int plateNumber;
    int slotNumber;
    char reservedLevel;
    int filled;
} ReserveInfo;

ReserveInfo reserveArr[40]; //list of subscribed cars and their slots
int reserveIndex = 0; //end of the reserve array 

ParkingLot parkA[10]; 
ParkingLot parkB[10]; 
ParkingLot parkC[10]; 
ParkingLot parkD[10]; 

int wait_queue[16]; 
int head_car_queue = 0;
int tail_car_queue = 0;
int car_queue_size = 0;



out_msg msg_queue[MSG_QUEUE_SIZE_X];
int head_msg_queue = 0;
int tail_msg_queue = 0;
int msg_queue_size = 0;

char level = 'A'; //current car's level
int slot_number = 0; //current car's slot number
int total_empty_slots = 40;
int empty_reserved_slots = 0;

typedef enum {EMPTY_SPACE_M, PARKING_SPACE_M, PARKING_FEE_M, RESERVED_M} message_state;
message_state curr_message_state = EMPTY_SPACE_M;


// These are used to disable/enable UART interrupts before/after
// buffering functions are called from the main thread. This prevents
// critical race conditions that result in corrupt buffer data and hence
// incorrect processing
inline void disable_rxtx( void ) { PIE1bits.RC1IE = 0;PIE1bits.TX1IE = 0;}
inline void enable_rxtx( void )  { PIE1bits.RC1IE = 1;PIE1bits.TX1IE = 1;}



/* **** Ring-buffers for incoming and outgoing data **** */
// These buffer functions are modularized to handle both the input and
// output buffers with an input argument.
typedef enum {INBUF = 0, OUTBUF = 1} buf_t;

#define BUFSIZE 128        /* Static buffer size. Maximum amount of data */
uint8_t inbuf[BUFSIZE];   /* Preallocated buffer for incoming data */
uint8_t outbuf[BUFSIZE];  /* Preallocated buffer for outgoing data  */
uint8_t head[2] = {0, 0}; /* head for pushing, tail for popping */
uint8_t tail[2] = {0, 0};
int headX = 0;
int tailX = 0;
/* Check if a buffer had data or not */
#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_isempty( buf_t buf ) { 
    headX = head[buf];
    tailX = tail[buf];
    return (head[buf] == tail[buf])?1:0; 
}
/* Place new data in buffer */
//#pragma interrupt_level 2 // Prevents duplication of function
void buf_push( uint8_t v, buf_t buf) {
    if (buf == INBUF) inbuf[head[buf]] = v;
    else outbuf[head[buf]] = v;
    head[buf]++;
    if (head[buf] == BUFSIZE) head[buf] = 0;
    if (head[buf] == tail[buf]) { ; }
}
/* Retrieve data from buffer */
#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_pop( buf_t buf ) {
    uint8_t v;
    if (buf_isempty(buf)) { 
        return 0; 
    } else {
        if (buf == INBUF) v = inbuf[tail[buf]];
        else{ v = outbuf[tail[buf]];
        outbuf[tail[buf]] = 0;}
        tail[buf]++;
        if (tail[buf] == BUFSIZE) tail[buf] = 0;
        return v;
    }
}

/* **** ISR functions **** */
void receive_isr() {
    PIR1bits.RC1IF = 0;      // Acknowledge interrupt
    buf_push(RCREG1, INBUF); // Buffer incoming byte
}
void transmit_isr() {
    PIR1bits.TX1IF = 0;    // Acknowledge interrupt
    // If all bytes are transmitted, turn off transmission
    if (buf_isempty(OUTBUF) == 1){
        while (TXSTA1bits.TRMT == 0){} // For reading last byte
        TXSTA1bits.TXEN = 0; 
    }
    // Otherwise, send next byte
    else TXREG1 = buf_pop(OUTBUF);
}

void adc_isr() {
    PIR1bits.ADIF = 0; // Acknowledge interrupt -> adc value is written in ADRESH and ADRESL
    
    read_adc_value = 1; //set flag
}


void __interrupt(high_priority) high_priority_isr() {
    if (PIR1bits.RC1IF) receive_isr();
    if (PIR1bits.TX1IF) transmit_isr();
    if (PIR1bits.ADIF) adc_isr();
    if (INTCONbits.TMR0IF) { //every 50 ms
        
        INTCONbits.TMR0IF = 0;
        TMR0H = 0x3c;
        TMR0L = 0xd0;
        
        if (waiting == 0) { //if in running mode
            //increase counters
            general_counter++;
            message_counter++;
        }
        
        if (adc_counter == 100) { //reached 500 ms -> check adc
            adc_counter = 1; //reset the counter
            check_adc_flag = 1;
        }
        else {
            adc_counter++;
        }
        
        if (waiting == 0) {
            update_display = 1; //change the digit shown 
        }
              
    }
    if (INTCONbits.RBIF) {
        
        INTCONbits.RBIF = 0;
        uint8_t portb = PORTB; //read to clear the flag
        
        if (((portb & 0x10) == 0x10) && (prev_rb4 == 0x00)) { //button released
            if (display_total_money) {
                display_total_money = 0; //display empty slots instead
            }
            else {
                display_total_money = 1;
            }
        } 
        
        prev_rb4 = portb & 0x10; //store the value to check later
        
    }
}

/* **** Initialization functions **** */
void init_ports() {
     //Port B pin 0: buffer overflow, pin1: buffer underflow, pin2: syntax err
    TRISB = 0xf0; //rb4-7 set to input for rb change interrupt
    TRISJ = 0x00; //used for 7-segment display
    TRISH = 0x10; //RH4 input (potentiometer), RH0-3 output (display)
    LATB = 0x00;
    LATH = 0x08; //digit-0 at first 
    LATJ = 0x00; //7-segment display unlit in waiting mode
}

// Choose SPBRG from Table 20.3
#define SPBRG_VAL (21)
void init_serial() { //CHANGE!!!!!!!!
    // We will configure EUSART1 for 115200 baud (uluc hoca -> 57600)
    // SYNC = 0, BRGH = 0, BRG16 = 1. Simulator does not seem to work 
    // very well with BRGH=1
    
    TXSTA1bits.TX9 = 0;    // No 9th bit
    TXSTA1bits.TXEN = 0;   // Transmission is disabled for the time being
    TXSTA1bits.SYNC = 0; 
    TXSTA1bits.BRGH = 0;
    RCSTA1bits.SPEN = 1;   // Enable serial port
    RCSTA1bits.RX9 = 0;    // No 9th bit
    RCSTA1bits.CREN = 1;   // Continuous reception
    BAUDCON1bits.BRG16 = 1;

    SPBRGH1 = (SPBRG_VAL >> 8) & 0xff;
    SPBRG1 = SPBRG_VAL & 0xff;
}

//initially all parking lots are empty and unreserved
void init_parking_lots() {
    for (int i=0; i < 10; i++) { 
        parkA[i].plateNumber = -1;
        parkA[i].isReserved = -1;
        parkB[i].plateNumber = -1;
        parkB[i].isReserved = -1;
        parkC[i].plateNumber = -1;
        parkC[i].isReserved = -1;
        parkD[i].plateNumber = -1;
        parkD[i].isReserved = -1;
    }
    
}

void init_timer_and_pie() {
    T0CON = 0x00;
    T0CONbits.TMR0ON = 1;
    T0CONbits.PSA = 1; //no prescaler needed
    
    //set for 5 ms timer
    TMR0H = 0x3c;
    TMR0L = 0xd0;
    
    RCONbits.IPEN = 0; //no priority
    
    PIE1bits.RC1IE = 1;//receiving enabled
    PIE1bits.TX1IE = 0;//transmission disabled
    
    PIE1bits.ADIE = 1;    //enable end of A/D conversion interrupt
    
    INTCON = 0x00;
    INTCONbits.TMR0IE = 1;
    INTCONbits.PEIE = 1; //enable peripheral interrupt for serial and adc
    INTCONbits.GIE = 1; 
    INTCONbits.RBIE = 1; //enable portb on change interrupt
    INTCONbits.RBIF = 0;
    uint8_t portb_tmp = PORTB; //make sure RBIF is clear
}


/* **** Packet task **** */
#define PKT_MAX_SIZE 20 // Maximum packet size. Syntax errors can be large!
#define PKT_HEADER 0x24  // Marker for start-of-packet: $
#define PKT_END 0x23     // Marker for end-of-packet: #

// State machine states for packet reception. 
typedef enum {PKT_WAIT_HDR, PKT_GET_BODY, PKT_WAIT_ACK} pkt_state_t;
pkt_state_t pkt_state = PKT_WAIT_HDR;
uint8_t pkt_body[PKT_MAX_SIZE]; // Packet data
uint8_t pkt_bodysize;           // Size of the current packet
// Set to 1 when packet is received. Must be set to 0 once packet is processed
uint8_t pkt_valid = 0;
uint8_t pkt_id = 0; // Incremented for every valid packet reception

/* The packet task is responsible from monitoring the input buffer, identify
 * the start marker 0x00, and retrieve all subsequent bytes until the end marker
 * 0xff is encountered. This packet will then be processed by the calc_task()
 * to parse and execute the arithmetic expression. */
void packet_task() {
    disable_rxtx();
    // Wait until new bytes arrive
    if (!buf_isempty(INBUF)) {
        uint8_t v;
        switch(pkt_state) {
        case PKT_WAIT_HDR:
            v = buf_pop(INBUF); 
            if (v == PKT_HEADER) { //received '$'
                // Packet header is encountered, retrieve the rest of the packet
                pkt_state = PKT_GET_BODY;
                pkt_bodysize = 0;
            }
            break;
        case PKT_GET_BODY:
            v = buf_pop(INBUF);
            if (v == PKT_END) { //received '#'
                // End of packet is encountered, signal calc_task())
                pkt_state = PKT_WAIT_ACK;
                pkt_valid = 1;
            } else if (v == PKT_HEADER || (pkt_bodysize + 1 == PKT_MAX_SIZE)) { //our size control
                // Unexpected packet start. Abort current packet and restart
                pkt_bodysize = 0;
            } else 
                pkt_body[pkt_bodysize++] = v; //add popped element (v) to the pkt_body
            break;
        case PKT_WAIT_ACK:
            if (pkt_valid == 0) {
                // Packet processing seems to be finished, continue monitoring
                pkt_state = PKT_WAIT_HDR;
                pkt_id++;
            }
            break;
        }
    }
    enable_rxtx();
}


void read_packet() {
    if (pkt_valid == 0) return; //wait until there is a valid (completed) packet to process
   
    //GO COMMAND
    if (waiting && pkt_body[0] == G && pkt_body[1] == O) {
        waiting = 0; //go to running state
        pkt_valid = 0; //packet processing is finished
        return;
    }
    
    //END COMMAND
    if (pkt_body[0] == E && pkt_body[1] == N && pkt_body[2] == D) {
        finished = 1; //go to finished state
        pkt_valid = 0; //packet processing is finished
        return;
    } 
    
    //PARK COMMAND
    if (pkt_body[0] == P && pkt_body[1] == R && pkt_body[2] == K) {
        licence_to_int(pkt_body[3], pkt_body[4], pkt_body[5]); //store car's plate number in global licence_number
        execute_command('p', licence_number, -1); //no slot number for this command
    }
    
    //EXIT COMMAND
    else if (pkt_body[0] == E && pkt_body[1] == X && pkt_body[2] == T) {
        licence_to_int(pkt_body[3], pkt_body[4], pkt_body[5]); //store car's plate number in global licence_number
        execute_command('e', licence_number, -1); //no slot number for this command
    }
    
    //SUBSCRIPTION COMMAND
    else if (pkt_body[0] == S && pkt_body[1] == U && pkt_body[2] == B) {
        licence_to_int(pkt_body[3], pkt_body[4], pkt_body[5]); //store car's plate number in global licence_number
        level = pkt_body[6]; //stored in global level variable
        park_spot_to_int(pkt_body[7], pkt_body[8]); //store spot number in global park_spot
        execute_command('s', licence_number, park_spot);
    }
    
    pkt_valid = 0; //packet processing is finished
}

// ************ OUTPUT PART ************

uint8_t pkt_out[PKT_MAX_SIZE]; // Output Packet data
uint8_t pkt_outsize = 0;       // Size of the current packet

void fill_outbuffer() {
    for (int i = 0; i < pkt_outsize; ++i) { //send each byte of the out packet
        disable_rxtx(); //pause interrupts
        buf_push(pkt_out[i], OUTBUF);
        enable_rxtx();
    }
    //buf_push(0x23, OUTBUF); // Delete after testing
    for (int i = 0; i < PKT_MAX_SIZE; ++i) { //clear the output array
        pkt_out[i] = 0;
    }
}

void send_packet() {
    out_msg msg;
    if (msg_queue_size > 0) msg = pop_msg_queue(); //get the first message waiting in message queue
    else curr_message_state = EMPTY_SPACE_M; //send empty message if queueu is empty

    if (msg.msg_type == 'S') curr_message_state = PARKING_SPACE_M;
    else if (msg.msg_type == 'F') curr_message_state = PARKING_FEE_M;
    else if (msg.msg_type == 'R') curr_message_state = RESERVED_M;

    switch (curr_message_state) {
        case EMPTY_SPACE_M:
            pkt_out[0] = '$';
            pkt_out[1] = 'E';
            pkt_out[2] = 'M';
            pkt_out[3] = 'P';
            pkt_out[4] = total_empty_slots / 10 + 0x30; //first digit (0x30 is '0')
            pkt_out[5] = total_empty_slots % 10 + 0x30; //second digit
            pkt_out[6] = '#';
            pkt_outsize = 7;
            break;
        case PARKING_SPACE_M:            
            pkt_out[0] = '$';
            pkt_out[1] = 'S';
            pkt_out[2] = 'P';
            pkt_out[3] = 'C';
            pkt_out[4] = msg.licence_num / 100 + 0x30;
            pkt_out[5] = (msg.licence_num / 10) % 10 + 0x30;
            pkt_out[6] = msg.licence_num % 10 + 0x30;
            pkt_out[7] = msg.level;
            pkt_out[8] = msg.park_spot / 10 + 0x30;
            pkt_out[9] = msg.park_spot % 10 + 0x30;
            pkt_out[10] = '#';
            pkt_outsize = 11;
            break;
        case PARKING_FEE_M:            
            pkt_out[0] = '$';
            pkt_out[1] = 'F';
            pkt_out[2] = 'E';
            pkt_out[3] = 'E';
            pkt_out[4] = int_to_char(msg.licence_num, 100);
            pkt_out[5] = int_to_char(msg.licence_num, 10);;
            pkt_out[6] = int_to_char(msg.licence_num, 1);;
            pkt_out[7] = int_to_char(msg.total_fee, 100);
            pkt_out[8] = int_to_char(msg.total_fee, 10);
            pkt_out[9] = int_to_char(msg.total_fee, 1);
            pkt_out[10] = '#';
            pkt_outsize = 11;
            break;
        case RESERVED_M:            
            pkt_out[0] = '$';
            pkt_out[1] = 'R';
            pkt_out[2] = 'E';
            pkt_out[3] = 'S';
            pkt_out[4] = int_to_char(msg.licence_num, 100);
            pkt_out[5] = int_to_char(msg.licence_num, 10);
            pkt_out[6] = int_to_char(msg.licence_num, 1);
            pkt_out[7] = int_to_char(msg.total_fee, 10);
            pkt_out[8] = int_to_char(msg.total_fee, 1);
            pkt_out[9] = '#';
            pkt_outsize = 10;
            break;
        default:
            pkt_outsize = 0;
            break;
    }
    fill_outbuffer();
}
 
/* Output task function */
void output_task() {
    // The output task stays in this state forever after initialization
    disable_rxtx();
    // Check if there is any buffered output or ongoing transmission
    if (buf_isempty(OUTBUF) == 0 && TXSTA1bits.TXEN == 0) { 
        
        // If transmission is already ongoing, do nothing, 
        // the ISR will send the next char. Otherwise, send the 
        // first char and enable transmission
        TXSTA1bits.TXEN = 1;
        TXREG1 = buf_pop(OUTBUF);
    }
    enable_rxtx();
}

// ************ FUNCTIONS *************

//convert string licence plate number to int and store it in licence_number
void licence_to_int(uint8_t a, uint8_t b, uint8_t c) {
    licence_number = 0;
    licence_number += (a - '0') * 100;
    licence_number += (b - '0') * 10;
    licence_number += (c - '0');
}

//convert string park spot number to int and store it in licence_number
void park_spot_to_int(uint8_t a, uint8_t b) {
    park_spot = 0;
    park_spot += (a - '0') * 10;
    park_spot += (b - '0');
}

void push_car_queue(int licence_num) {
    if (car_queue_size == 16) { //queue is already full
        return;
    }
    wait_queue[tail_car_queue] = licence_num; //add car to the end
    tail_car_queue = (tail_car_queue + 1) % 16; //increment tail index
    car_queue_size++; //increase queue size
}

int pop_car_queue(void) {
    if (car_queue_size == 0) { //no car in queue
        return -1;
    }
    int licence = wait_queue[head_car_queue]; //get first car
    head_car_queue = (head_car_queue + 1) % 16; //increment head index
    car_queue_size--;
    return licence;
}

void push_msg_queue(out_msg msg) {
    int next_tail = (tail_msg_queue + 1) % MSG_QUEUE_SIZE_X; //move the tail
    if (next_tail == head_msg_queue) { //message queue is full
        return;
    }
    msg_queue[tail_msg_queue] = msg;
    tail_msg_queue = next_tail; //move tail
    msg_queue_size++;
}

out_msg pop_msg_queue() {
    out_msg empty = { .msg_type = 'X' }; //empty message
    if (head_msg_queue == tail_msg_queue) { //queue is empty
        return empty;
    }
    out_msg m = msg_queue[head_msg_queue];
    head_msg_queue = (head_msg_queue + 1) % MSG_QUEUE_SIZE_X; //move head
    msg_queue_size--;
    return m;
}


void park_the_car(int carToPark){
    ParkingLot* park_lot;
    int flag = 0; //has the car parked yet?
    
    
    //search fot the car in the subsribed cars list
    for (int i = 0; i < reserveIndex; ++i) {
        if (reserveArr[i].plateNumber == carToPark) { //found the car
            reserveArr[i].filled = 1; //fill the spot
            out_msg msg;
            msg.level = reserveArr[i].reservedLevel;
            msg.licence_num = carToPark;
            msg.park_spot = reserveArr[i].slotNumber+1;
            msg.msg_type = 'S'; //parking space message
            push_msg_queue(msg); //push message to queue
            total_empty_slots--;
            empty_reserved_slots--; //reserved slot is no longer empty
            empty_spaces[level_to_index(reserveArr[i].reservedLevel)]--; //decrease the level's empty space count
            return;
        }
    }
    
    //if the car is not subsribed check if there's any spot to park
    //car can't park in reserved slots even though they're empty
    if(total_empty_slots - empty_reserved_slots == 0){ //no lot to park
        push_car_queue(carToPark);
        return;
    }
    
    for(int i = 0; i < 10; i++){
        if(parkA[i].plateNumber == -1 && parkA[i].isReserved == -1){ //if lot is empty and not reserved
            parkA[i].plateNumber = carToPark;
            parkA[i].entranceTime = general_counter;
            slot_number = i+1; //since lots start from 1 instead of 0
            flag = 1; //parked successfully
            level = 'A';
            break;
        }
    }
    if (flag == 0) { //not parked yet 
        for (int i = 0; i < 10; i++) {
            if(parkB[i].plateNumber == -1 && parkB[i].isReserved == -1){ //if lot is empty and not reserved
                parkB[i].plateNumber = carToPark;
                parkB[i].entranceTime = general_counter;
                slot_number = i+1; //since lots start from 1 instead of 0
                flag = 1;
                level = 'B';
                break;
            }
        }
    }
    if (flag == 0) { //not parked yet 
        for (int i = 0; i < 10; i++) {
            if(parkC[i].plateNumber == -1 && parkC[i].isReserved == -1){ //if lot is empty and not reserved
                parkC[i].plateNumber = carToPark;
                parkC[i].entranceTime = general_counter;
                slot_number = i+1; //since lots start from 1 instead of 0
                flag = 1;
                level = 'C';
                break;
            }
        }
    }
    if (flag == 0) { //not parked yet 
        for (int i = 0; i < 10; i++) {
            if(parkD[i].plateNumber == -1 && parkD[i].isReserved == -1){ //if lot is empty and not reserved
                parkD[i].plateNumber = carToPark;
                parkD[i].entranceTime = general_counter;
                slot_number = i+1; //since lots start from 1 instead of 0
                flag = 1;
                level = 'D';
                break;
            }
        }
    }

    //send the parking message
    out_msg msg;
    msg.level = level;
    msg.licence_num = carToPark;
    msg.park_spot = slot_number;
    msg.msg_type = 'S';
    push_msg_queue(msg);
    total_empty_slots--;
    empty_spaces[level_to_index(level)]--; //decrease that level's empty space count
    return;
}

void reserve_the_lot(int carToReserve, int slotNumber){
    //do slotNumber-1 since slots are 1-indexed
    int successful_reservation = 0;
    switch (level){
        case 'A':
            if(parkA[slotNumber-1].plateNumber == -1 && parkA[slotNumber-1].isReserved == -1){ //check if the slot is available
                parkA[slotNumber-1].isReserved = carToReserve; 
                successful_reservation = 1;
                //add the car to the subscribed cars list
                reserveArr[reserveIndex].plateNumber = carToReserve;
                reserveArr[reserveIndex].slotNumber = slotNumber-1;
                reserveArr[reserveIndex].reservedLevel = 'A';   
                reserveArr[reserveIndex].filled = 0;
                reserveIndex++;
            }
            break;
        case 'B':
            if(parkB[slotNumber-1].plateNumber == -1 && parkB[slotNumber-1].isReserved == -1){ //check if the slot is available
                parkB[slotNumber-1].isReserved = carToReserve;
                successful_reservation = 1;
                //add the car to the subscribed cars list
                reserveArr[reserveIndex].plateNumber = carToReserve;
                reserveArr[reserveIndex].slotNumber = slotNumber-1;
                reserveArr[reserveIndex].reservedLevel = 'B'; 
                reserveArr[reserveIndex].filled = 0;
                reserveIndex++;
            }
            break;
        case 'C':
            if(parkC[slotNumber-1].plateNumber == -1 && parkC[slotNumber-1].isReserved == -1){ //check if the slot is available
                parkC[slotNumber-1].isReserved = carToReserve;
                successful_reservation = 1;
                //add the car to the subscribed cars list
                reserveArr[reserveIndex].plateNumber = carToReserve;
                reserveArr[reserveIndex].slotNumber = slotNumber-1;
                reserveArr[reserveIndex].reservedLevel = 'C';   
                reserveArr[reserveIndex].filled = 0;
                reserveIndex++;
            }
            break;
        case 'D':
            if(parkD[slotNumber-1].plateNumber == -1 && parkD[slotNumber-1].isReserved == -1){ //check if the slot is available
                parkD[slotNumber-1].isReserved = carToReserve;
                successful_reservation = 1;
                //add the car to the subscribed cars list
                reserveArr[reserveIndex].plateNumber = carToReserve;
                reserveArr[reserveIndex].slotNumber = slotNumber-1;
                reserveArr[reserveIndex].reservedLevel = 'D';   
                reserveArr[reserveIndex].filled = 0;
                reserveIndex++;
            }
            break;
    }

    //send subsribed message
    out_msg msg;
    if(successful_reservation){
        msg.total_fee = 50; 
        INTCONbits.TMR0IE = 0; //disable timer interrupt since total_money is shared variable
        total_money += 50;
        INTCONbits.TMR0IE = 1;
        empty_reserved_slots++; //slot is empty at first
    }
    else{ //unsuccessful reservation
        msg.total_fee = 0;
    }
    msg.licence_num = carToReserve; 
    msg.msg_type = 'R';
    push_msg_queue(msg); //push subsribe message to queue
    return;
}

int calculateFee(int entranceTime){
    //substract car's entrance time from current time to find fee
    int difference = (general_counter - entranceTime)/50; //divide by 50 instead of 250 since timer is 5 ms
    return difference + 1;
}

void exit(int carToExit){
    //check if it's a reserved car
    for (int i = 0; i < reserveIndex; ++i) {
        if (reserveArr[i].plateNumber == carToExit && reserveArr[i].filled == 1) { //to prevent duplicate exit
            reserveArr[i].filled = 0; 
            //prepare message
            out_msg msg;
            msg.licence_num = carToExit;
            msg.total_fee = 0; //fee is 0 for subscribed car exit
            msg.msg_type = 'F';
            push_msg_queue(msg);
            total_empty_slots++;
            empty_reserved_slots++; //reserved slot becomes empty
            empty_spaces[level_to_index(reserveArr[i].reservedLevel)]++; //increase that level's empty spaces
            return;
        }
    }
    
    int entranceTime = -1;
    char exit_car_level;
    int exit_car_slot;
    for(int i = 0; i < 10; i++){ //find the car that wants to exit
        if(parkA[i].plateNumber == carToExit){ 
            entranceTime = parkA[i].entranceTime; 
            exit_car_slot = i+1; //+1 for writing to message
            exit_car_level = 'A';
            parkA[i].plateNumber = -1; //empty the lot
            parkA[i].entranceTime = -1;
            break;
        }
        if(parkB[i].plateNumber == carToExit){ 
            entranceTime = parkB[i].entranceTime; 
            exit_car_slot = i+1; //+1 for writing to message
            exit_car_level = 'B';
            parkB[i].plateNumber = -1; //empty the lot
            parkB[i].entranceTime = -1;
            break;
        }
        if(parkC[i].plateNumber == carToExit){ 
            entranceTime = parkC[i].entranceTime; 
            exit_car_slot = i+1; //+1 for writing to message
            exit_car_level = 'C';
            parkC[i].plateNumber = -1; //empty the lot
            parkC[i].entranceTime = -1;
            break;
        }
        if(parkD[i].plateNumber == carToExit){ 
            entranceTime = parkD[i].entranceTime; 
            exit_car_slot = i+1; //+1 for writing to message
            exit_car_level = 'D';
            parkD[i].plateNumber = -1; //empty the lot
            parkD[i].entranceTime = -1;
            break;
        }
    }
    if(entranceTime == -1){ //couldn't find the car
        return;
    }
    total_empty_slots++;
    empty_spaces[level_to_index(exit_car_level)]++;
    int fee = calculateFee(entranceTime);
    INTCONbits.TMR0IE = 0; //disable interrupt since total_money is shared
    total_money += fee; //add to the total money collected
    INTCONbits.TMR0IE = 1;
    
    //output message
    out_msg msg;
    msg.licence_num = carToExit;
    msg.total_fee = fee;
    msg.msg_type = 'F';
    push_msg_queue(msg);

    //check the queue
    if(car_queue_size > 0 && (total_empty_slots - empty_reserved_slots > 0)){ 
        //check if there's a car in the queue and that there's at least 1 empty non-reserved lot
        int popped_car = pop_car_queue(); //get the first car at the queue
        total_empty_slots--;
        empty_spaces[level_to_index(exit_car_level)]--;
        if (exit_car_level == 'A'){
            parkA[exit_car_slot-1].plateNumber = popped_car;
            parkA[exit_car_slot-1].entranceTime = general_counter;
        }
        if (exit_car_level == 'B'){
            parkB[exit_car_slot-1].plateNumber = popped_car;
            parkB[exit_car_slot-1].entranceTime = general_counter;
        }
        if (exit_car_level == 'C'){
            parkC[exit_car_slot-1].plateNumber = popped_car;
            parkC[exit_car_slot-1].entranceTime = general_counter;
        }
        if (exit_car_level == 'D'){
            parkD[exit_car_slot-1].plateNumber = popped_car;
            parkD[exit_car_slot-1].entranceTime = general_counter;
        }
        
        //park message
        out_msg space_msg;
        space_msg.level = exit_car_level;
        space_msg.licence_num = popped_car; 
        space_msg.park_spot = exit_car_slot;
        space_msg.msg_type = 'S';
        push_msg_queue(space_msg);
    }
}

//call the functions that execute simulator's commands
void execute_command(char message, int plateNumber, int slotNumber){
    if(message == 'p'){
        park_the_car(plateNumber);
    }

    if(message == 'e' && total_empty_slots <= 40){ //don't call exit if it's empty
        exit(plateNumber);
    }
    if(message == 's'){
        if(total_empty_slots - empty_reserved_slots > 0){ //don't call exit there are no empty non-reserved lots
            reserve_the_lot(plateNumber, slotNumber);
        }
    }
}

int int_to_char(int val, int divider) {
    return (val / divider) % 10 + 0x30;
}

//empty space count info
void send_empty_spaces_packet() {
    pkt_out[0] = '$';
    pkt_out[1] = 'E';
    pkt_out[2] = 'M';
    pkt_out[3] = 'P';
    pkt_out[4] = total_empty_slots / 10 + 0x30;
    pkt_out[5] = total_empty_slots % 10 + 0x30;
    pkt_out[6] = '#';
    pkt_outsize = 7;

    fill_outbuffer(); 
}

//maps from levels (char) to indices (used for accessing empty_spaces)
int level_to_index(char level_char) {
    switch (level_char) {
        case 'A':
            return 0;
        case 'B':
            return 1;
        case 'C':
            return 2;
        case 'D':
            return 3;
    }
}


//convert decimal number's digits to an array of LATJ values that should be used in the display 
void int_to_7segment(int number){
	
	int numbers[10] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};

    int digits[10] = {0}, i = 0;
    while (number > 0) { //traverse each digit of the number
        digits[i++] = number % 10;
        number /= 10;
    }
    for (i = 0; i < 4; ++i)
    {
    	display_digits[i] = numbers[digits[i]];
    }
}

//light the current digit to display with the correct value 
void set_digit_pin(){
            switch (digit) {
            case 0:
                LATH = 0x08; //rightmost (least significant) bit
                LATJ = display_digits[0];
                break;
            case 1:
                LATH = 0x04;
                LATJ = display_digits[1];
                break;
            case 2:
                LATH = 0x02;
                LATJ = display_digits[2];
                break;
            case 3:
                LATH = 0x01; //leftmost (most significant) bit
                LATJ = display_digits[3];
                break;
        }
        digit = (digit+1)%4;
}

void init_adc() {
    ADCON0 = 0x31; //
    ADCON1 = 0x02;
    ADCON2 = 0xaa;
}

void main(void) {
    init_parking_lots(); //set all parking spots to empty
    init_timer_and_pie();
    init_serial();
    init_ports();
    init_adc();
    
    while (1) {
        packet_task(); //check for new packets, push to array
        read_packet(); //read from input array and execute commands
        if (waiting == 0) {
            output_task(); //start the transmission if necessary (new batch of data)
        }
    
        if (waiting == 0 && message_counter >= 20) { //every 100 ms
            if (msg_queue_size > 0) {
                send_packet(); //send message (parking info, fee, reserve)         
            }
            else { //if no message waiting in queue
                send_empty_spaces_packet(); 
            }
            message_counter = 0;
        }
        
        if (check_adc_flag) {
            check_adc_flag = 0; //clear flag
            ADCON0bits.GO = 1; //start adc conversion
        }
        
        if (read_adc_value) { //adc interrupt was received
            read_adc_value = 0;
            uint16_t result = (ADRESH << 8) | ADRESL; //read 10-bit adc value (right-aligned)
            //map to levels
            if (result >= 0 && result < 256) display_level = level_to_index('A');
            else if (result >= 256 && result < 512) display_level = level_to_index('B');
            else if (result >= 512 && result < 768) display_level = level_to_index('C');
            else if (result >= 768 && result <= 1023) display_level = level_to_index('D');
        }
        
        if (waiting == 0 && update_display == 1) { //change digit shown
            update_display = 0;
            if (display_total_money) {
                int_to_7segment(total_money); //set values to total_money digits
            }
            else { //display empty spaces
                int empty = empty_spaces[display_level];
                int_to_7segment(empty); //set values to display_level's empty slot count
            }
            set_digit_pin(); //configure the display to the digit that will be shown
        }
      
    }
    
    return;
}