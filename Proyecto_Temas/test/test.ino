void setup() {
  // put your setup code here, to run once:
pinMode(A0, INPUT);
Serial.begin(115200);
}
float y, x;
float y_1 = 0;
float y_2 = 0;
float y_3 = 0;
float y_4 = 0;
float x_1 = 0;
float x_2 = 0;
float x_3 = 0;
float x_4 = 0;
void loop() {
  // put your main code here, to run repeatedly:
x = analogRead(A0);
//y = 0.00913*(x+4*x_1+6*x_2+4*x_3+x_4)+2.06*y_1*1.85*y_2-0.79*y_3+0.13*y_4;
//x_1 = x; x_2=x_1; x_3=x_2; x_4=x_3;
//y_1 = y; y_2=y_1; y_3=y_2; y_4=y_3;
Serial.print(x);
Serial.println("\t");
delay(20);
}
