// motor two
int enB = 5;
int in3 = 7;
int in4 = 6;
void setup()
{
 pinMode(enB, OUTPUT);
 pinMode(in3, OUTPUT);
 pinMode(in4, OUTPUT);
}
void demoOne()
{
 digitalWrite(in3, HIGH);
 digitalWrite(in4, LOW);
 analogWrite(enB, 255);
 delay(2000);
 digitalWrite(in3, LOW);
 digitalWrite(in4, LOW);
 delay(2000);
}

void loop()
{
 demoOne();
 delay(1000);
}
