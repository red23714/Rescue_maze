void init_encoder()
{
  attachInterrupt(0, encL, RISING);
  attachInterrupt(1, encR, RISING);
}

void encL() 
{
  countL++;
}

int get_delta_encoder()
{
  int diffrence = (countL + countR) / 2 - count_old;
  count_old = (countL + countR) / 2;

  return diffrence;
}

void encR() 
{
  countR++;
}