/*
Shitty drone detection robot code we're gonna strap a glizzy to the side of this thing and I'm not talking about a fuckking hotdog
Jacob Lamoureux February 6 2025
University of Guelph
*/

const int ena_Z = 11;
const int dir_Z = 10;
const int pul_Z = 9;
const int limit_Z = 8;

const int ena_Y = 6;
const int dir_Y = 5;
const int pul_Y = 3;
const int limit_Y = 2;

int z;
int y;


void setup() {
  pinMode(ena_Z, OUTPUT);
  pinMode(dir_Z, OUTPUT);
  pinMode(pul_Z, OUTPUT);
  pinMode(limit_Z, INPUT);

  pinMode(ena_Y, OUTPUT);
  pinMode(dir_Y, OUTPUT);
  pinMode(pul_Y, OUTPUT);
  pinMode(limit_Y, INPUT);
  
  digitalWrite(ena_Z, LOW);
  digitalWrite(ena_Y, LOW);

  //limit_Y is triggered at 0 (LOW)
  //limit_Z is triggered at 1 (HIGH)

  digitalWrite(dir_Z, LOW);                     //LOW = Counter clockwise rotation
  digitalWrite(dir_Y, LOW);                     //LOW = Counter clockwise rotation
  for(int i = 0; i<1600; i++){
    digitalWrite(pul_Z, HIGH);
    digitalWrite(pul_Y, HIGH);
    delayMicroseconds(100);
    digitalWrite(pul_Z, LOW);
    digitalWrite(pul_Y, LOW);
    delayMicroseconds(100);
  }
  //Serial.begin(115200);
}

void loop() {
  digitalWrite(ena_Z, HIGH);
  digitalWrite(ena_Y, HIGH);
  /*
  Serial.print("Limit_Z = ");
  Serial.print(digitalRead(limit_Z));
  Serial.print("\t");

  Serial.print("Limit_Y = ");
  Serial.print(digitalRead(limit_Y));
  Serial.println();
  */


}
