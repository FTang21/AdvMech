void Shoot() {
  analogWrite(PWMspeed, 255);
  shooting = true;
  if (!shot) {
    activateServo();
    shot = true;
    delayTime = millis();
  } else {
    if(millis() - delayTime > 5000) {
      shot = false;
      delayTime = 0;
    }
  }
}

void NoShoot() {
  analogWrite(PWMspeed, 0);
  shooting = false;
  shot = false;
}
