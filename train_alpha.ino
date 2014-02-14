#include <Keypad.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


/**
 * General purpose constants.
 */
#define MAX_PIN_OUT            255
#define MAX_PIN_IN             1023
#define RELAY_ON               LOW
#define RELAY_OFF              HIGH
#define TRIGGER_DELAY          5  // cycles to delay the trigger from next response

/**
 * Constants that describe the mode of operation.
 */
#define MODE_OFF               0
#define MODE_LOGICAL           1   // the logic mode - T1 departs/arrives at S1
#define MODE_SHIFTING          2   // the shifting mode - T1 departs from S1, arrives at next trains departure
#define MODE_NATURAL           3   // the natural mode - aka, random, some passing, some shifting
#define MODE_MANUAL            4   // the manual mode - controllable via switches and buttons
#define MODE_COUNT             5   // count of modes

/**
 * Constants that describe track layouts.
 */
#define LAYOUT_TRIDENT         1
#define LAYOUT_STACKED         2
#define LAYOUT_TWO_SIDE        3
#define LAYOUT_THREE_SIDE      4
#define LAYOUT_COUNT           4

/**
 * I thought this was somewhat ingenious - to call the
 * left of right side of a switch, just call
 * digitalWrite(SWITCH_1+LEFT);
 */
#define LEFT                   0
#define RIGHT                  1

/**
 * LCD Address and pin information, for use in displaying information.
 */
#define LCD_ADDR               0x27 // may vary based on device
#define LCD_BL                 3
#define LCD_EN                 2
#define LCD_RW                 1
#define LCD_RS                 0
#define LCD_D4                 4
#define LCD_D5                 5
#define LCD_D6                 6
#define LCD_D7                 7
// initialize library
LiquidCrystal_I2C              lcd(LCD_ADDR,LCD_EN,LCD_RW,LCD_RS,LCD_D4,LCD_D5,LCD_D6,LCD_D7);

/**
 * Keypad information.
 */
const byte KEYPAD_ROWS         = 4; // Four rows
const byte KEYPAD_COLS         = 4; // Three columns
const char KEYPAD_KEYS[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'Y','0','N','D'}
};
byte KEYPAD_ROW_PINS[KEYPAD_ROWS] = { 38, 39, 40, 41 };
byte KEYPAD_COL_PINS[KEYPAD_COLS] = { 42, 43, 44, 45 }; 
Keypad kpd = Keypad( makeKeymap(KEYPAD_KEYS), KEYPAD_ROW_PINS, KEYPAD_COL_PINS, KEYPAD_ROWS, KEYPAD_COLS );
#define KEYPAD_HOLD_TIME       500
#define KEYPAD_DEBOUNCE_TIME   100


/**
 * Pins for the analog input provided by the train sensors which will
 * assist in maintaining state.
 */
#define SENSOR_1               0   // sensor 1, immediately after yard
#define SENSOR_2               1   // sensor 2, when next train is due to depart
#define SENSOR_3               2   // sensor 3, when train is pulling into station
#define SENSOR_4               3   // sensor 3, when train is pulling into station

/**
 * Pins for switch relays.  These are just the first - they're in 
 * pairs of two, one for left and one for right.  Just the first pin
 * is listed.  This uses the Seeedstudio relay board, so HIGH is off
 * and LOW is powered.
 */
#define SWITCH_1               22  // the first (right) pin for switch 1 (first yard switch)
#define SWITCH_2               24  // the first (right) pin for switch 2 (second yard switch)
#define SWITCH_3               26  // the first (right) pin for switch 3 (spur switch)
#define SWITCH_IGNORE          999 // just a placeholder so we can set the proper sized array.

/**
 * Pins for section power relays. Send a HIGH signal to these pins to
 * activate the relay to power the sections.  Note, that mainline is not
 * included in the sections array.  This is to standardize the Section 1 =
 * Index 0 format, and because Mainline is really an edge case only included
 * because of manual control.  Otherwise an emergency it's always on.
 */
#define SECTION_MAINLINE       999  // this pin will allow power to go to mainline (section 5, as this will not be controlled often)
#define SECTION_ONE            32  // this pin will power yard section 1
#define SECTION_TWO            33  // this pin will power yard section 2
#define SECTION_THREE          34  // this pin will power yard section 3
#define SECTION_SPUR           35  // this pin will power the spur (section 4)

/**
 * These are going to be counts largely used in control structures in the
 * application.  By modifying these, you can control how the application
 * works (ie, logic setup with 2 trains, 1, etc).
 */
#define SENSOR_COUNT           2   // number of sensors available (up to 3)
#define TRAIN_COUNT            3   // number of trains available
#define SWITCH_COUNT           3   // number of switches available
#define SECTION_COUNT          3   // number of sections controlled (not incl. mainline)
#define BUTTON_COUNT           3   // number of sections controlled (not incl. mainline)

/**
 * Arrays of pins, grouped largely by function.  These will be set in setup(),
 * or a function called from it.
 */
const int SENSORS[SENSOR_COUNT]           = { SENSOR_1, SENSOR_2 }; // array of sensors
const int SWITCHES[SWITCH_COUNT]          = { SWITCH_1, SWITCH_2, SWITCH_3 }; // array of switches
const int SECTIONS[SECTION_COUNT]         = { SECTION_ONE, SECTION_TWO, SECTION_THREE }; // array of sections (note no mainline);
const String MODES[MODE_COUNT]            = { "Depowered", "Logical", "Cycling", "Natural", "Manual" }; // array of switches
const String LAYOUTS[LAYOUT_COUNT]        = { "Trident Yard", "Stacked Yard", "Two Sidings", "Three Sidings" }; // array of switches

/**
 * Path related variables, to determine how to finagle switches.
 */
int PATHS[3][2]                           = { { SWITCH_1+RIGHT, SWITCH_IGNORE },
                                              { SWITCH_1+LEFT, SWITCH_2+RIGHT },
                                              { SWITCH_1+LEFT, SWITCH_2+LEFT }};
int SWITCH_POSITIONS[SWITCH_COUNT]        = { 0, 0, 0 }; // array of switches
boolean spur_available = false;
int spur_switch_number = 3;
boolean spur_enabled = false;
int yard_selected = 0;
                                      

/**
 * These are application controlled vars that will control information about the
 * sensors.
 */
int sensor_standards[SENSOR_COUNT];
int sensor_delays[SENSOR_COUNT];

/**
 * Application controlled vars that keep track of various details of the components
 * (trains, sections, locations, etc).
 */
boolean prepared          = false;
boolean running           = false;
boolean input_locked      = false;
boolean layout            = 0;
boolean mode              = 0;

/**
 * Track the power status of the trains/sections.
 */
boolean train_power[TRAIN_COUNT];
boolean section_power[SECTION_COUNT];

/**
 * Track train statuses
 */
int train_stations[TRAIN_COUNT]     = {1, 2, 3};
int train_locations[TRAIN_COUNT]    = {0, 0, 0}; // 0: yard/approaching, 1: leaving yard, 2: mainline
int last_departure                  = 0;


/**
 * Kick it all off - setup pins, sample light, and start cycle.
 */
void setup() {
  Serial.begin(9600);
  // start off by telling arduino how to handle i/o
  initialize_pins();
  
  // setup the lcd details
  initialize_lcd();
  
  // setup the lcd details
  initialize_keypad();
  
  // start off by telling arduino how to handle i/o
  initialize_vars();

  // take the first sample of the lights
  sample_light();
  
  // print welcome message
  print_welcome();
  
  // prepare the program
  prepare_program();
  
  // confirm and start program
  confirm_program();
}


/**
 * For each loop, we want to check the input from the sensors,
 * print the status to the LEDs, and handle any required power switching
 * and turnout control.
 */
void loop() {
  // check the status of the track sensors
  check_sensors();
  
  // delay and collect input
  input_delay(100);
}


/**
 * Resposible for setting the I/O status of each required
 * pin.  This relies heavily on the arrays created at the top
 * of this file.
 */
void initialize_pins() {
  
  // initialize light sensors
  for(int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSORS[i], OUTPUT);
  }
  
  // initialize switch relay pins
  for(int i = 0; i < SWITCH_COUNT; i++) {
    // set as high (stupid seeedstudio relay board...)
    digitalWrite(SWITCHES[i]+LEFT, LOW);
    digitalWrite(SWITCHES[i]+RIGHT, LOW);

    // set mode
    pinMode(SWITCHES[i]+LEFT, OUTPUT);
    pinMode(SWITCHES[i]+RIGHT, OUTPUT);
  }

  // initialize powered section relay pins
  for(int i = 0; i < SECTION_COUNT; i++) {
    digitalWrite(SECTIONS[i], HIGH);
    pinMode(SECTIONS[i], HIGH);
  }
}


/**
 * Setup the LCD.
 */
void initialize_lcd() {
  // initialize lcd
  lcd.begin (20,4); // initialize with number of cols and rows
  lcd.setBacklightPin(LCD_BL,POSITIVE);
  lcd.setBacklight(HIGH);
}


/**
 * Setup the keypad event listener and other details.
 */
void initialize_keypad() {
  kpd.addEventListener(keypad_event);
  kpd.setHoldTime(KEYPAD_HOLD_TIME);
  kpd.setDebounceTime(KEYPAD_DEBOUNCE_TIME);
}


/**
 * Setup the initial variables.
 */
void initialize_vars() {
  for(int i = 0; i < TRAIN_COUNT; i++) {
    train_power[i] = false;
  }
  
  for(int i = 0; i < SECTION_COUNT; i++) {
    section_power[i] = false;
  }
  
  for(int i = 0; i < SENSOR_COUNT; i++) {
    sensor_standards[i] = 0;
    sensor_delays[i] = 0;
  }
}


/**
 * Print a happy welcome message
 */
void print_welcome() {
  lcd.clear();
  lcd_print_line(1, "Welcome to the");
  lcd_print_line(2, "TrainMaster3000");
  lcd_print_line(4, "Preparing system...");
  delay(1000);
}


/**
 * Start with the welcome message and system status, then move into
 * the regular system display.
 */
boolean prepare_program() {
  prepared = false;
  
  boolean success = prompt_program_details();
  while(!success) {
    success = prompt_program_details();
  }
  
  prepared = true;
  return true;
}

boolean prompt_program_details() {
  layout = 0;
  spur_available = false;
  
  // prompt for layout
  lcd.clear();
  lcd_print_line(1, "1) Trident Yard");
  lcd_print_line(2, "2) Stacked Yard");
  lcd_print_line(3, "3) Opposing Yards");
  lcd_print_line(4, "4) Three Sidings");
  
  // wait for input
  while(layout == 0) {
    char key = kpd.getKey();
    if(key) {
      if(key == '1')  {
        layout = 1;
      } else if(key == '2') {
        layout = 2;
      } else if(key == '3') {
        layout = 3;
      } else if(key == '4') {
        layout = 4;
      }        
    }
  }
  
  // determine if spur exists (not on 3 siding track)
  if(layout != 4) {
    lcd.clear();
    lcd_print_line(1, LAYOUTS[layout-1]);
    lcd_print_line(2, "Is there a spur?");
    lcd_print_line(3, "1) Yes");
    lcd_print_line(4, "2) No");
    
    boolean spur_selection_made = false;      
    while(!spur_selection_made) {
      char key = kpd.getKey();
      if(key) {
        if(key == '1') {
          spur_available = true;
        } else if(key == 'N') {
          layout = 0;
          return false;
        }
        
        spur_selection_made = true;
      }
    }
  }

  // confirmation
  return true;
}


/**
 * 
 */
void confirm_program() {
    // print system status
  lcd.clear();
  lcd_print_line(1, (String) TRAIN_COUNT + " trains on a");
  lcd_print_line(2, LAYOUTS[layout-1] + (String)(spur_available ? "+Spur" : ""));
  lcd_print_line(3, "layout, " + (String) SENSOR_COUNT + " sensors");
  lcd_print_line(4, "Starting (0 to test)");
  
  // wait for another bit
  boolean debug_completed = false;
  int delay_counter = 2000;
  
  while(!debug_completed && delay_counter > 0) {
    char key = kpd.getKey();
    if(key) {
      if(key == '0') {
        debug_system();
        debug_completed = true;
      }
    }
    
    delay_counter -= 5;
    delay(5);
  }

  lcd_init_contents();
}


/**
 * This function cycles through the modes.  It will change, on the fly,
 * the mode that the mode that the system is running on.
 */
void set_mode(int selected_mode) {
  
  mode = selected_mode;
  running = false;
  depower_section(0);
  
  // reset train positions
  for(int i = 0; i < TRAIN_COUNT; i++) {
    train_stations[i] = i+1;
    train_locations[i] = 0;
  }
  
  // redraw the entire lcd, maybe a lot has changed
  lcd_init_contents();
}


/**
 * All this really needs to do is set running to false.  The triggers will handle it from
 * there.  Really, all this means is that no track sections are to be powered.  We want to make
 * sure we continue tracking train positions
 */
void start_cycle() {
  if(!running && mode != MODE_OFF) {
    running = true;
    if(mode != MODE_MANUAL) {
      lcd_print_action("Powering train 1");
      power_train(1);
    }
    lcd_init_contents();
  }
}


/**
 * All this really needs to do is set running to false.  The triggers will handle it from
 * there.  Really, all this means is that no track sections are to be powered.  We want to make
 * sure we continue tracking train positions
 */
void stop_cycle() {
  if(mode == MODE_MANUAL) {
    depower_section(0);
  } else {
    // halt or turn off
    if(running) {
      running = false;
      depower_section(0);
      lcd_init_contents();
    } else if(mode != MODE_OFF) {
      mode = MODE_OFF;
      lcd_init_contents();
    } else if(mode == MODE_OFF) {
      prepare_program();
      confirm_program();
    }
  }
}


/**
 * Use the values in train_status to determine whether or not a train is
 * in motion.  In motion means that they are pulling out of a station or
 * on the mainline.  Their status goes dark as they're pulling into the
 * yard.
 */
void lcd_init_contents() {
  // clear the contents of the screen, first and foremost
  lcd.clear();
  
  // Line 1: Mode of operation
  lcd.setCursor(0,0);
  lcd.print("Mode: ");
  
  // Line 2: Track status
  lcd.setCursor(0,1);        // go to start of 2nd line
  lcd.print("Path: ");
      
  // Line 3: Train status
  lcd.setCursor(0,2);
  lcd.print("Stat: ");
  
  // fill in the details
  lcd_print_mode();
  lcd_print_path();
  lcd_print_status();
}


/**
 * Print to a whole line, clearing first.
 */
void lcd_print_line(int line, String contents) {
  lcd.setCursor(0, line-1);
  lcd.print("                    ");
  lcd.setCursor(0, line-1);
  lcd.print(contents);
}

/**
 * Print the mode to the proper position on the LCD screen.
 */
void lcd_print_mode() {
  // clear the current status displayed
  lcd.setCursor(6,0);
  lcd.print("          ");
  lcd.setCursor(6,0);
  
  // write the new status
  lcd.print(MODES[mode]);
  
  if(!running && mode != MODE_OFF) {
    lcd.print(" Halted");
  }
}


/**
 * Print the status of the trains
 */
void lcd_print_path() {
  // clear the current train status displayed
  lcd.setCursor(6,1);
  lcd.print("          ");
  lcd.setCursor(6,1);

  lcd.print("Undefined");
}


/**
 * Print the status of the trains and powered sections.
 */
void lcd_print_status() {
  // clear the current train status displayed
  lcd.setCursor(6,2);
  lcd.print("              ");
  
  // reset cursor
  lcd.setCursor(6,2);
  
  if(mode != MODE_MANUAL) {
    // print each trains status as a number (yard section), M (mainline), or S (spur)
    for(int i = 0; i < TRAIN_COUNT; i++) {
      if(i > 0) {
        lcd.print("/");
      }
      
      if(train_power[i]) {
        lcd.print("M");
      } else {
        lcd.print((String) train_stations[i]);
      }
    }
    
    // space in between trains and sections
    lcd.print(" ");

  }
  
  // print each sections power status
  for(int i = 0; i < SECTION_COUNT; i++) {
    if(i > 0) {
      lcd.print("/");
    }
    
    if(section_power[i]) {
      lcd.print("+");
    } else {
      lcd.print("-");
    }
  }
}


/**
 * Print an action to the 4th line
 */
void lcd_print_action(String action) {
  // clear the current train status displayed
  lcd_print_line(4, action);
}


/**
 * Loop through each sensor and determine if there's anything
 * that needs to be done, if there's a trigger, or if we're
 * in cooldown mode.
 */
void check_sensors() {
  // sample each of the sensors
  for(int i = 0; i < SENSOR_COUNT; i++) {
    // sample the sensor
    int val = analogRead(SENSORS[i]);
    
    // delay is in effect, handle accordingly
    if(sensor_delays[i] > 0) {
      sensor_delays[i]--;
      
      // re-trigger and reset the delay if the light drops down beneath 90% after recovery
      if((float)val / (float)sensor_standards[i] * 100.00 < 90) {
        sensor_delays[i] = TRIGGER_DELAY;
      }
    } else {
      // trigger the sensors action handler
      if((float) val / (float) sensor_standards[i] * 100.00 < 80) {
        trigger_sensor_action(i);
        sensor_delays[i] = TRIGGER_DELAY;
      }
    }
  }
}


/**
 * This is the gatekeeper for the sensor actions.  If one is triggered,
 * handle it in here.
 */
void trigger_sensor_action(int sensor) {
  if(sensor == 0) {
    // the first action is marking that the train has left the yard
    trigger_yard_departure();
    
    // trigger the remaining sections if we don't the right amount of sensors
    if(SENSOR_COUNT < 2) trigger_mainline_sensor();
    if(SENSOR_COUNT < 3) trigger_yard_entry();
  } else if(sensor == 1) {
    // the second action is going to handle the train rolling over the mainline sensor
    trigger_mainline_sensor();
    
    // trigger the remaining section if we don't have a third sensor
    if(SENSOR_COUNT < 3) trigger_yard_entry();
  } else {
    trigger_yard_entry();
  }
  
  // print the status of the trains and sections
  lcd_print_status();
}


/**
 * Depower the track section the train departed from.
 */
void trigger_yard_departure() {
  // depower departing trains section
  depower_section(train_stations[last_departure-1]);
  
  // write the status
  if(mode == MODE_MANUAL) {
    lcd_print_action("Train has left yard");
  } else {
    lcd_print_action("T" + (String) last_departure + " leaving yard");
  }
}


/**
 * A train is rolling over the mainline sensor, lets figure out what to do with it, and
 * using the mode, figure out how to handle the transfer.  This is one of the bulkier
 * sensor triggers.
 */
void trigger_mainline_sensor() {  
  // determine the next train to leave the yard
  int next_train = last_departure + 1 <= TRAIN_COUNT ? last_departure + 1 : 1;

  if(mode == MODE_LOGICAL) {
    // each train returns to its original station.  open its original, power the next
    handle_logical_transfer(next_train);
  } else if(mode == MODE_SHIFTING) {
    // each train goes to the next trains section
    handle_shifting_transfer(next_train);
  } else if(mode == MODE_NATURAL) {
    // figure out the next train to go
    Serial.println("Natural Transfer");
    int random_train = random(1, TRAIN_COUNT+1);
    Serial.println("Next train: " + (String) next_train + " from " + (String) train_stations[next_train-1]);
    handle_logical_transfer(random_train);
//    
//    if(random_train == last_departure) {
//      // same train is going out. power section, power train
//      lcd_print_line(2, "Passthrough transfer");
//      handle_passing_transfer();
//    } else {
//      // a different train is going out, pick a transfer type.
//      int transfer_type = (int) random(1, 2);
//      
//      if(transfer_type == 1) {
//        lcd_print_line(2, "Logical transfer");
//        handle_empty_bay_transfer(random_train);
//      } else {
//        lcd_print_line(2, "Cycling transfer");
//        handle_shifting_transfer(random_train);
//      }
//    }
  }
  
  trigger_yard_entry();
}


/**
 * This is called when a train is entering the yard.  This touches train power status only.
 */
void trigger_yard_entry() {
  int arriving_train = last_departure - 1;
  
  if(arriving_train == 0) {
    arriving_train = TRAIN_COUNT;
  }
  
  // indicate train has returned
  train_power[arriving_train-1] = false;
}


/**
 * Arriving train returns to its original departure point, next train departs.
 */
void handle_logical_transfer(int next_train) {  
  // log to LCD
  lcd_print_action("T" + (String) last_departure + " in S" + (String) train_stations[last_departure-1] + ", T" + (String) next_train + " out S" + (String) train_stations[next_train-1]);

  // process action
  train_locations[next_train-1] = 1;
  set_switch_positions(train_stations[last_departure-1]);
  power_train(next_train);
}


/**
 * Arriving train returns to its original departure point, next train departs.
 */
void handle_empty_bay_transfer(int next_train) {
  // find an empty bay
  int empty_bay = 0;
  
  int sections[3] = { 1, 2, 3 };
  
  Serial.println("TRAIN INFORMATION");
  for(int i = 0; i < TRAIN_COUNT; i++) {
    // check each trains section and remove from section list
    sections[train_stations[i]-1] == 0;
  }
  
  for(int i = 0; i < 3; i++) {
    if(sections[i] > 0) {
      empty_bay = sections[i];
      break;
    }
  }

  train_stations[last_departure-1] = empty_bay;
  
  Serial.println("empty bay transfer");
  Serial.println("Sending running train to section " + (String)empty_bay);
  
  // log to LCD
  lcd_print_action("T" + (String) last_departure + " in S" + (String) empty_bay + ", T" + (String) next_train + " out S" + (String) train_stations[next_train-1]);

  // process action
  set_switch_positions(empty_bay);
  power_train(next_train);
}


/**
 * Arriving train returns to the departing trains station.
 */
void handle_shifting_transfer(int next_train) {
  train_stations[last_departure-1] = train_stations[next_train-1];
  train_locations[last_departure-1] = 1;
  
  // log to lcd
  lcd_print_action("T" + (String) last_departure + " in S" + (String) train_stations[last_departure-1] + ", T" + (String) next_train + " out S" + (String) train_stations[next_train-1]);  
  
  // process action
  set_switch_positions(train_stations[next_train-1]);
  power_train(next_train);
}


/**
 * This train is going to pass through its original section.
 */
void handle_passing_transfer() {
  // log to lcd
  lcd_print_action("T" + (String) last_departure + " passthru S" + (String) train_stations[last_departure-1]);
  
  set_switch_positions(train_stations[last_departure-1]);
  power_train(last_departure);
}


/**
 * The gatekeeper for switching switch positions.
 */
void set_switch_positions(int pos) {
  // check switch position and set switch 1
  if(PATHS[pos-1][0] < 100 && SWITCH_POSITIONS[0] != PATHS[pos-1][0]) {
    digitalWrite(PATHS[pos-1][0], HIGH);
    SWITCH_POSITIONS[0] = PATHS[pos-1][0];
    input_delay(250);
    digitalWrite(PATHS[pos-1][0], LOW);
  }
  
  // check switch position and set switch 2
  if(PATHS[pos-1][1] < 100 && SWITCH_POSITIONS[1] != PATHS[pos-1][1]) {
    digitalWrite(PATHS[pos-1][1], HIGH);
    SWITCH_POSITIONS[1] = PATHS[pos-1][1];
    input_delay(250);
    digitalWrite(PATHS[pos-1][1], LOW);
  }
  
  if(spur_available && mode != MODE_MANUAL) {
    boolean spur_it = random(0, 2) == 1; // generate 0 or 1
    int pin = SWITCHES[spur_switch_number-1] + (spur_it ? LEFT : RIGHT);
    
    if(SWITCH_POSITIONS[SWITCHES[spur_switch_number-1]] != pin) {
      digitalWrite(pin, HIGH);
      SWITCH_POSITIONS[SWITCHES[spur_switch_number-1]] = pin;
      input_delay(250);
      digitalWrite(pin, LOW);
    }
  }
}


/**
 * The gatekeeper for setting the spur switch
 */
void toggle_spur() {
  if(spur_available) {
    // pick left or right dependent on current position
    int pin = SWITCHES[spur_switch_number-1]+LEFT;
    if(SWITCH_POSITIONS[SWITCHES[spur_switch_number-1]] == pin) {
      pin = SWITCHES[spur_switch_number-1]+RIGHT;
    }
    
    // write to the pin
    digitalWrite(pin, HIGH);
    SWITCH_POSITIONS[SWITCHES[spur_switch_number-1]] = pin;
    input_delay(250);
    digitalWrite(pin, LOW);
  }
}

/**
 * Depowers a train section.
 */
void depower_section(int section) {
  // depower section and set status
  if(section == 0) {
    for(int i = 0; i < SECTION_COUNT; i++) {
      digitalWrite(SECTIONS[i], RELAY_OFF);
      section_power[i] = false;
    }
  } else {
    digitalWrite(SECTIONS[section-1], RELAY_OFF);
    section_power[section-1] = false;
  }
  
  lcd_print_status();
}


/**
 * Send power to a track section.
 */
void power_train(int train) {
  if(running) {
    // power the section
    digitalWrite(SECTIONS[train_stations[train-1]-1], RELAY_ON);
    
    // mark the last departed train and station/train status
    section_power[train_stations[train-1]-1] = true;
    train_power[train-1] = true;
    last_departure = train;
    
    // power the trains
    lcd_print_status();    
  }
}


/**
 * Send power to a track section.
 */
void power_section(int section) {
  if(running) {
    // power the section
    digitalWrite(SECTIONS[section-1], RELAY_ON);
    
    // mark the last departed train and station/train status
    section_power[section-1] = true;
    
    // power the trains
    lcd_print_status();    
  }
}


/**
 * Take a sample of the light and use it as the standard for each sensor
 */
void sample_light() {
  for(int i = 0; i < SENSOR_COUNT; i++) {
    int val = analogRead(SENSORS[i]);
    sensor_standards[i] = val;
  }
}


/**
 * Input delay - delays for the total count of ms in 10ms increments, so we can check for input in
 * the interim.
 */
void input_delay(int ms) {
  long time = millis();
  while(millis() < time + ms) {
    kpd.getKey();
  }
}


/**
 * Handle keypad input.
 */
void keypad_event(KeypadEvent key) {
  static char last_held = NO_KEY;
  
  if(prepared && !input_locked) {
    switch (kpd.getState()) {
      case HOLD:
        last_held = key;
        
        if(mode == MODE_MANUAL) {
          // for manual mode operation, we want to power a section
          if(key == '1' || key == '2' || key == '3') {
            char * key_p = & key;
            power_section(atoi(key_p));
            lcd_print_action("Powering section " + (String) key);
          }
        }
        break;

      case RELEASED:
        // collect the input, ignore if it was a held key (we want those events fired immediately on *held*)
        char input = key;
        if(last_held != NO_KEY) {
          input = NO_KEY;
          last_held = NO_KEY;
        }
        
        if(input != NO_KEY) {
          if(input == 'A') {
            set_mode(MODE_LOGICAL);
          } else if(input == 'B') {
            set_mode(MODE_SHIFTING);
          } else if(input == 'C') {
            set_mode(MODE_NATURAL);
          } else if(input == 'D') {
            set_mode(MODE_MANUAL);
          } else if(input == 'Y') {
            start_cycle();
          } else if(input == 'N') {
            stop_cycle();
          }
          
          if(mode == MODE_MANUAL) {
            if(key == '1' || key == '2' || key == '3' || key == '0') {
              char * key_p = & key;
              int path = atoi(key_p);
              
              if(path == 0) {
                toggle_spur();
                lcd_print_action("Toggling spur");
              } else {
                set_switch_positions(path);
                lcd_print_action("Opening section  " + (String) path);
              }
            }
          }
        
//          lcd_print_action("Button " + (String) key + " pressed");
        }      
        break;
    }
  }
}






void debug_system() {
  boolean debug_complete = false;
  
  while(!debug_complete) {
    debug();
    
    char key = kpd.getKey();
    while(!key) {
      key = kpd.getKey();
      if(key == 'Y') {
        Serial.println("Test completed");
        debug_complete = true;
      }
      delay(5);
    }
  }
  
  
}


/**
 * Debug the various workings of the system.
 */
void debug() {
  lcd.clear();
  lcd_print_line(1, "    SYSTEM TEST     ");
  
  // test light sensors
  lcd_print_line(2, "Testing Sensors...");
  for(int i = 0; i < 100; i++) {
    int input_1 = analogRead(SENSORS[0]);
    int input_2 = analogRead(SENSORS[1]);
    int input_3 = analogRead(SENSORS[2]);
    lcd_print_line(3, "1: " + (String) input_1 + " 2: " + (String) input_2 + " 3: " + (String) input_3);
    delay(100);
  }
  lcd_print_line(3, "");

  lcd_print_line(2, "Testing Switches...");
  for(int i = 0; i < 1; i++) {
    for(int j = 0; j < SWITCH_COUNT; j++) {
      // high to switch right
      lcd_print_line(3, "Switch " + (String) (j+1) + " Left");
      digitalWrite(SWITCHES[j]+LEFT, HIGH);
      delay(250);
      digitalWrite(SWITCHES[j]+LEFT, LOW);
      lcd_print_line(3, "");
      delay(250);

      // high to switch left
      lcd_print_line(3, "Switch " + (String) (j+1) + " Right");
      digitalWrite(SWITCHES[j]+RIGHT, HIGH);
      delay(250);
      digitalWrite(SWITCHES[j]+RIGHT, LOW);
      lcd_print_line(3, "");
      delay(250);
    }
  }

  lcd_print_line(2, "Testing Sections...");
  for(int i = 0; i < 1; i++) {
    for(int j = 0; j < SECTION_COUNT; j++) {
      lcd_print_line(3, "Section " + (String) (j+1) + " On");
      digitalWrite(SECTIONS[j], RELAY_ON);
      delay(500);
      lcd_print_line(3, "");
      digitalWrite(SECTIONS[j], RELAY_OFF);
      delay(250);
    }
  }
  
  lcd_print_line(2, "Test Complete");
  lcd_print_line(3, "Press Green to exit");
  lcd_print_line(4, "or Red to retest.");
}




