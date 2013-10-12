/* Written by Christian Smith, Oct 2005*/
/*
  Library of powercube commands.
*/

#ifndef _POWERCUBE_COMMANDS_H_
#define _POWERCUBE_COMMANDS_H_

struct module{
  canHandle handle;
  long int canID;
};

#define PC_DATA_TYPE_FLOAT          0
#define PC_DATA_TYPE_INT8           1
#define PC_DATA_TYPE_UINT8          2
#define PC_DATA_TYPE_UINT16         3
#define PC_DATA_TYPE_INT16          4
#define PC_DATA_TYPE_UINT32         5
#define PC_DATA_TYPE_INT32          6
#define PC_DATA_TYPE_FLOAT_UINT16   7
#define PC_DATA_TYPE_INT32_UINT16   8

/*
Returns the data type of a message
 */
int pc_get_data_type(unsigned char commandID, unsigned char paramID);


/* 
   The following commands can be used to set the char[8] data message's 
   byte 2-7 to contain data of different types. The returned value can
   be used as dlc
*/
unsigned int set_data_int(unsigned char msg[8], long int *val);

unsigned int set_data_uint32(unsigned char msg[8], unsigned long int *val);

unsigned int set_data_int16(unsigned char msg[8], short int *val);

unsigned int set_data_uint16(unsigned char msg[8], unsigned short int *val);

unsigned int set_data_float(unsigned char msg[8], float *val);

unsigned int set_data_float_uint16(unsigned char msg[8], float *fval, unsigned short int *ival);

unsigned int set_data_int_uint16(unsigned char msg[8], long int *ival32, unsigned short int *ival16);


/* 
   The following commands can be used to get the char[8] data message's 
   byte 2-7 and store the results as different types
*/
void get_data_int(unsigned char msg[8], long int *val);

void get_data_uint32(unsigned char msg[8], unsigned long int *val);

void get_data_int16(unsigned char msg[8], short int *val);

void get_data_uint16(unsigned char msg[8], unsigned short int *val);

void get_data_float(unsigned char msg[8], float *val);

void get_data_float_uint16(unsigned char msg[8], float *fval, 
			   unsigned short int *ival);

void get_data_int_uint16(unsigned char msg[8], long int *ival32,
			 unsigned short int *ival16);


/*
  Sends 'halt' command to all modules on bus h
*/
void pc_halt_all_on_bus(canHandle h);

/*
  Sends 'reset' command to all modules on bus h
*/
void pc_reset_all_on_bus(canHandle h);

/*
  Sends 'home' command to all modules on bus h
*/
void pc_home_all_on_bus(canHandle h);

/*
  Sends 'set baudrate' command to all modules on bus h
  Baudrate must be one of 
  {PC_BAUDRATE_250K, PC_BAUDRATE_500K, PC_BAUDRATE_1M}
*/
void pc_set_baud_all_on_bus(canHandle h, unsigned char baudrate);

/*
  Sends 'save pos' command to all modules on bus h
*/
void pc_save_all_on_bus(canHandle h);

/*
  Sends 'watchdog refresh' command to all modules on bus h
*/
void pc_watch_refresh_all_on_bus(canHandle h);

/*
  Sends 'synchronize' command to all modules on bus h
*/
void pc_sync_all_on_bus(canHandle h);

/*
  Sends 'halt' command to all modules on all four buses (h1, h2, h3, and h4)
*/
void pc_halt_all(canHandle h1, canHandle h2, canHandle h3, canHandle h4);

/*
  Sends 'reset' command to all modules on all four buses (h1, h2, h3, and h4)
*/
void pc_reset_all(canHandle h1, canHandle h2, canHandle h3, canHandle h4);

/*
  Sends 'watchdog refresh' command to all modules on all 
  four buses (h1, h2, h3, and h4)
*/
void pc_watch_refresh_all(canHandle h1, canHandle h2, 
			  canHandle h3, canHandle h4);

/*
  Sends 'set baudrate' command to all modules on all 
  four buses (h1, h2, h3, and h4)
  Baudrate must be one of 
  {PC_BAUDRATE_250K, PC_BAUDRATE_500K, PC_BAUDRATE_1M}
*/
void pc_set_baud_all(canHandle h1, canHandle h2, 
		     canHandle h3, canHandle h4, 
		     unsigned char baudrate);

/*
  Sends 'home' command to all modules on all four buses (h1, h2, h3, and h4)
*/
void pc_home_all(canHandle h1, canHandle h2, canHandle h3, canHandle h4);

/*
  Resets a single module
*/
void pc_reset_module(struct module _mod);

/*
  Halts a single module
*/
void pc_halt_module(struct module _mod);

/*
  Homes a single module
*/
void pc_home_module(struct module _mod);

/*
  Sends a command to a single module.
*/
void pc_send_command_to_module(struct module _mod, unsigned char msg[8], 
			       unsigned int dlc);
void pc_send_command_to_module_nowait(struct module _mod, unsigned char msg[8], unsigned int dlc);

/*
  Returns the state word of a single module
*/
unsigned long int pc_get_status(struct module _mod);

/*
  Prints the module state given a module state word
 */
void pc_print_module_state(unsigned long int _cube_state);

/*
  Requests a value from a single module.
*/
void pc_request_value_from_module(struct module _mod, unsigned char msg[8]);
void pc_request_value_from_module_nowait(struct module _mod, unsigned char msg[8]);

/*
  Listen for module response. Returns the number of the module that 
  responded. Waits until response arrives, or time out (100 ms).
  Returns negative value for error (e.g time out)
*/
int pc_listen_for_response(canHandle h, void *msg);
int pc_listen_for_response_nowait(canHandle h, void *msg);

char* pc_par2str(unsigned char parnum);

void pc_display_message(unsigned char *msg);

pthread_t pc_start_watchdog_thread(struct module *_mod);
void pc_enable_watchdog(struct module _mod);

void pc_kill_watchdog(pthread_t pth);

// opens can channels, sets default params, and goes on bus
void pc_open_init_onBus(canHandle h[4]);

// initializes CAN link to cubes  
void pc_initialize_powercubes(struct module cube[6], canHandle h[4]);

// Sets all powercubes and CAN channels to 250kBaud
void pc_setAll250k(canHandle h[4]);

// Home and reset all modules
// After homing, module 3 is set to -20 degrees to be within legal initial limits
void pc_home_and_reset_modules(struct module cube[6]);

// set angle limits
void pc_set_angle_limits(struct module cube[6],
			 float cubelimit_lower[6],
			 float cubelimit_upper[6]);

#endif // #ifndef _POWERCUBE_COMMANDS_H_

