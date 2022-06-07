#define pwmPin 3
#define sensor 2   // Interruptions for RPM

//int something = (100 * 25.5 / 182.4) * 10;
int pwmOut;
double input, output, setPoint;
double kp = 2, ki = 1, kd = 5;
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double error, lastError, cumError, rateError;

volatile int counter = 0;                   // volatile is storaged in RAM

void setup() {
  Serial.begin(57600);
  attachInterrupt(digitalPinToInterrupt(sensor), rmpCounter, RISING);    // pin2 interruption

  setPoint = 1000;
}
 
void loop() {
  delay(999);
  input = counter*(60/7);                   // RPM -> 60 / number of blades = 7, values -> 0 : 1800
  output = computePID(input);
  
  pwmOut = (output * 0.1 * 25.5 / 182.4) * 10;
  analogWrite(pwmPin,255);

  Serial.print(pwmOut);
  Serial.print(",");
  Serial.print(input);
  Serial.println(" RPM");
  counter = 0;
}
 
void rmpCounter() {                         // Function for interruption
    counter++;
}

double computePID(double inp) {
  currentTime = millis();                                 // Get current time
  elapsedTime = (double)(currentTime - previousTime);
        
  error = setPoint - input;
  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;
 
  double output = kp*error + ki*cumError + kd*rateError;
 
  lastError = error;anterior
  previousTime = currentTime;
 
  return output;
}
