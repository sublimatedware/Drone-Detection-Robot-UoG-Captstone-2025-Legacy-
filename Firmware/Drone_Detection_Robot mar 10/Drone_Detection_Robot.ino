/*
Shitty drone detection robot code we're gonna strap a glizzy to the side of this thing and I'm not talking about a fuckking hotdog
Jacob Lamoureux February 6 2025
University of Guelph
*/
//#indlude <Firmata.h>

const int ena_Z = 11;
const int dir_Z = 10;
const int pul_Z = 9;
const int limit_Z = 8;

const int ena_Y = 6;
const int dir_Y = 5;
const int pul_Y = 3;
const int limit_Y = 2;

int z = 0;
int y = 0;

byte serialIn = 0;

unsigned long currentTime = 0;

int blinkDelay = 100;

byte entered = 0;

void setup() {
  
  pinMode(13, OUTPUT);

  pinMode(ena_Z, OUTPUT);
  pinMode(dir_Z, OUTPUT);
  pinMode(pul_Z, OUTPUT);
  pinMode(limit_Z, INPUT);                  

  pinMode(ena_Y, OUTPUT);
  pinMode(dir_Y, OUTPUT);
  pinMode(pul_Y, OUTPUT);
  pinMode(limit_Y, INPUT);

//#############################################################################################################

  digitalWrite(ena_Z, LOW);                     //motors are ENABLED on LOW
  digitalWrite(ena_Y, LOW);

  //limit_Y is triggered at 0 (LOW)
  //limit_Z is triggered at 1 (HIGH)

  if ((digitalRead(limit_Z) == 1) && (digitalRead(limit_Y) == 0)){                                        //backs off of limits if initiated triggering limits
    digitalWrite(dir_Z, LOW);                    //LOW = Counter Clockwise rotation away from limit
    digitalWrite(dir_Y, HIGH);                     //HIGH = Clockwise rotation away from limit
    for(int i = 0; i<400; i++){
      digitalWrite(pul_Z, HIGH);
      digitalWrite(pul_Y, HIGH);
      delayMicroseconds(2000);
      digitalWrite(pul_Z, LOW);
      digitalWrite(pul_Y, LOW);
      delayMicroseconds(2000);
    }
  }


  digitalWrite(dir_Z, HIGH);                    //HIGH = Clockwise rotation toward limit
  digitalWrite(dir_Y, LOW);                     //LOW = Counter clockwise rotation towards limit

  do{
    if (digitalRead(limit_Z) == 0){             //Z limit triggered at 1
      digitalWrite(pul_Z, HIGH);  
    }

    if (digitalRead(limit_Y) == 1){             //Y limit triggered at 0
      digitalWrite(pul_Y, HIGH);  
    }
    delayMicroseconds(2000);
    if (digitalRead(limit_Z) == 0){              //crap i have to do because of the delay
      digitalWrite(pul_Z, LOW);
    }

    if (digitalRead(limit_Y) == 1){
      digitalWrite(pul_Y, LOW);
    }
    delayMicroseconds(2000);
  }
  while((digitalRead(limit_Y) == 1) || (digitalRead(limit_Z) == 0));

//#############################################################################################################

  Serial.begin(115200);
}

void loop() {

  digitalWrite(ena_Z, HIGH);
  digitalWrite(ena_Y, HIGH);

  

  if(Serial.available() > 0){
    serialIn = Serial.read();
    Serial.write(serialIn + 4);
    //blinkDelay = serialIn;
  }
/*
  currentTime=millis();
  if (((millis()%blinkDelay) == 0) && (entered == 0)){
    digitalWrite(13,!digitalRead(13));
    entered = 1;
    currentTime = millis();
    //Serial.println(millis());
  }

  if (millis() != currentTime)
    entered = 0;
  }
 /* 

  Serial.print("Limit_Z = ");
  Serial.print(digitalRead(limit_Z));
  Serial.print("\t");

  Serial.print("Limit_Y = ");
  Serial.print(digitalRead(limit_Y));

  Serial.print("Z in = ");
  Serial.print(z);
  Serial.println();
*/
}
