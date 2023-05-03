void activateServo() {
  for(pos = 140; pos >= 0; pos--) {
    myservo.write(pos);
    delay(10);
  }
  delay(1500);
  for(pos = 0; pos < 140; pos++) {
    myservo.write(pos);
    delay(10);
  }
  delay(500);
}
