#include <Arduino.h>
#include <SPI.h>
#include <MergCBUS.h>
#include <Message.h>
#include <EEPROM.h>

#define GREEN_LED     A5     //merg green led port
#define YELLOW_LED     2     //merg yellow led port
#define PUSH_BUTTON   A4     //std merg push button
#define CANPORT       10
#define INTPIN         8

#define N_UNCOUPLERS  4

int uncoupler[N_UNCOUPLERS] = { 3, 5, 6, 9 };
unsigned int  onTime = 5000;
unsigned long nextTransition = 0;

unsigned char state = 0;
unsigned char nextState = 0;

// GPIO pin support - maximum of 8 supported
int gpio[] = { 4, 7, A3, A2, A1, A0 };
int n_gpio = sizeof(gpio) / sizeof(int);
unsigned char gpio_mask = 0, gpio_pullup = 0, gpio_invert = 0;
bool gpio_state[8];

/**
   Node Variables:
      1         Coupler active millisecond MSB
      2         Coupler active millisecond LSB
      3         GPIO mask. Bit set for output, bit clear for input
      4         GPIO pullups - valid for inputs bit set if pullup required
      5         GPIO invert - set bit to make active low input/output on
*/
#define NUM_NODE_VARS  5    //the node variables
#define NUM_EVENTS     32   //supported events
#define NUM_EVENT_VARS 1    //supported event variables
#define NUM_DEVICES    1    //one device number
#define MODULE_ID      56   //module id
#define MANUFACTURER_ID 165 //manufacturer id
#define MIN_CODE       0    //min code version
#define MAX_CODE       1    //max code version

MergCBUS cbus = MergCBUS(NUM_NODE_VARS, NUM_EVENTS, NUM_EVENT_VARS, NUM_DEVICES);


void myUserFunc(Message *msg, MergCBUS *mcbus)
{
  /* getting standard on/off events */
  boolean onEvent;

  if (mcbus->eventMatch()) {
    onEvent = mcbus->isAccOn();
    int unc = mcbus->getEventVar(msg, 1); // Get first event variable

    if (unc >= N_UNCOUPLERS)
    {
      // It's a GPIO pin rather than an uncoupler
      int pin = unc - N_UNCOUPLERS;
      if (pin > n_gpio)
      {
        return;
      }
      if ((gpio_mask & (1 << pin)) != 0) // Pin must be an output
      {
        if ((gpio_invert & (1 << pin)) != 0)
        {
          onEvent = !onEvent;
        }
        digitalWrite(gpio[pin], onEvent ? HIGH : LOW);
      }
    }
    else if (onEvent)
    {
      if (state == 0)
      {
        digitalWrite(uncoupler[unc], HIGH);
        state = 1 << unc;
        nextTransition = millis() + onTime;
      }
      else if (nextState == 0)
      {
        nextState = 1 << unc;
      }
    }
    else
    {
      if (state & (1 << unc))
      {
        digitalWrite(uncoupler[unc], LOW);
        state = 0;
        if (nextState)
        {
          state = nextState;
          nextState = 0;
          for (int c = 0; c < N_UNCOUPLERS; c++)
          {
            if (state & (1 << c))
            {
              digitalWrite(uncoupler[c], HIGH);
            }
          }
          nextTransition = millis() + onTime;
        }
      }
    }
  }
}

void myUserFuncDCC(Message * msg, MergCBUS * mcbus)
{
}

/**
   Send the On or Off event for the GPIO read
*/
void send_gpio_event(int gpio_pin, bool state)
{
  if ((gpio_invert & (1 << gpio_pin)) != 0)
  {
    state = !state;
  }
  if (state)
    cbus.sendOnEvent(true, gpio_pin);
  else
    cbus.sendOffEvent(true, gpio_pin);
}

void nodeVariableWrite(int ind, int val)
{
  switch (ind)
  {
    case 1:
      onTime = (val << 8) | (onTime & 0xff);
      break;
    case 2:
      onTime = val | (onTime & 0xff00);
      break;
    case 3:
      gpio_mask = val;
      break;
    case 4:
      gpio_pullup = val;
      break;
    case 5:
      gpio_invert = val;
      break;
  }
}


void setup() {
  Serial.begin(9600);
  //Configuration data for the node
  cbus.getNodeId()->setNodeName("UNCP", 4);        //node name
  cbus.getNodeId()->setModuleId(MODULE_ID);            //module number
  cbus.getNodeId()->setManufacturerId(MANUFACTURER_ID);//merg code
  cbus.getNodeId()->setMinCodeVersion(MIN_CODE);       //Version 1
  cbus.getNodeId()->setMaxCodeVersion(MAX_CODE);
  cbus.getNodeId()->setProducerNode(true);
  cbus.getNodeId()->setConsumerNode(true);
  cbus.setPushButton(PUSH_BUTTON);//set the push button ports
  cbus.setStdNN(999); //standard node number

  //used to manually reset the node. while turning on keep the button pressed
  //this forces the node for slim mode with an empty memory for learned events and devices
  if (digitalRead(PUSH_BUTTON) == LOW) {
    Serial.println("Setup new memory");
    cbus.setUpNewMemory();
    cbus.saveNodeFlags();
    cbus.setNodeVariable(1, 0);
    cbus.setNodeVariable(2, 200);
    cbus.setNodeVariable(3, 0);
    cbus.setNodeVariable(4, 0);
    cbus.setNodeVariable(5, 0);
  }

  cbus.setLeds(GREEN_LED, YELLOW_LED); //set the led ports

  cbus.setUserHandlerFunction(&myUserFunc);//function that implements the node logic
  cbus.setDCCHandlerFunction(&myUserFuncDCC);
  cbus.setNodeVarHandlerFunction(&nodeVariableWrite);
  cbus.initCanBus(CANPORT, CAN_125KBPS, MCP_16MHz, 20, 30);  //initiate the transport layer
  cbus.setFlimMode();

  pinMode(INTPIN, INPUT);

  for (int i = 0; i < N_UNCOUPLERS; i++)
  {
    pinMode(uncoupler[i], OUTPUT);
    digitalWrite(uncoupler[i], LOW);
  }


  onTime = cbus.getNodeVar(1) * 256 + cbus.getNodeVar(2);
  gpio_mask = cbus.getNodeVar(3);
  gpio_pullup = cbus.getNodeVar(4);
  gpio_invert = cbus.getNodeVar(5);

  for (int i = 0; i < n_gpio; i++)
  {
    pinMode(gpio[i], (gpio_mask & ( 1 << i)) ? OUTPUT : ((gpio_pullup & (1 << i)) ? INPUT_PULLUP : INPUT));
    gpio_state[i] = false;
  }
}

void loop()
{
  cbus.cbusRead();
  cbus.run(); //do all cbuslogic

  // Custom loop code
  if (nextTransition && millis() > nextTransition)
  {
    for (int c = 0; c < N_UNCOUPLERS; c++)
    {
      if (state & (1 << c))
      {
        digitalWrite(uncoupler[c], LOW);
      }
    }
    if (nextState)
    {
      state = nextState;
      nextState = 0;
      for (int c = 0; c < N_UNCOUPLERS; c++)
      {
        if (state & (1 << c))
        {
          digitalWrite(uncoupler[c], HIGH);
        }
      }
      nextTransition = millis() + onTime;
    }
    else
    {
      state = 0;
      nextTransition = 0;
    }
  }

  for (int i = 0; i < n_gpio; i++)
  {
    if ((gpio_mask & ( 1 << i)) == 0)
    {
      bool state = digitalRead(gpio[i]);
      if (state != gpio_state[i])
      {
        gpio_state[i] = state;
        send_gpio_event(i, state);
      }
    }
  }


  //debug memory
  if (digitalRead(PUSH_BUTTON) == LOW)
  {
    cbus.dumpMemory();
  }
}
