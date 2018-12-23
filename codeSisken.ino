int motorPin = 5; 
int sensorPin = 2; 

volatile byte pulses; 
unsigned long timeold;
unsigned int pulsesperturn = 12;
void setVal();
double output = 0;
double error;
double sampleTime = 1;
double Kp = 300, Ki = 100, Kd = 1;            
double b0;             
double b1;             
double b2;             
double a1;             
double state[3];       
double Ts;             
static unsigned long previousTime = 0;
double feedback, setpoint = 800;
double pwmValue = 50;

void counter()
{
   pulses++;
}
void setup()
{
   Serial.begin(9600);
   pinMode(motorPin,OUTPUT);
   pinMode(sensorPin, INPUT);
   attachInterrupt(0, counter, FALLING);
   pulses = 0;
   feedback = 0;
   timeold = 0;
}
void loop()
{
      if (millis() - timeold >= 1000) {
      detachInterrupt(0);
      feedback = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
      timeold = millis();
      pulses = 0;
      attachInterrupt(0, counter, FALLING);
   }
      setVal();
      unsigned long now = millis();
      unsigned long elapsedTime = now - previousTime;
      if(elapsedTime >= Ts) {
        error = setpoint - feedback;
        output = a1*state[2] + b0*error + b1*state[0] + b2*state[1];
        state[1] = state[0]; 
        state[0] = error;    
        state[2] = output;   
        previousTime = now;
      }
      pwmValue = output / 2.95;
      analogWrite(motorPin,pwmValue);
      Serial.print(feedback);
      Serial.print(" ");
      Serial.println(error);
   
}

void setVal(){
      Ts = sampleTime;
      b0 = Kp + (Ki*(Ts/2)) + (Kd/Ts);
      b1 = (-Kp + (Ki*(Ts/2)) - ((2*Kd)/Ts));
      b2 = Kd/Ts;
      a1 = 1;
}

