void init_encoder()
{
  attachInterrupt(0, encL, RISING);
  attachInterrupt(1, encR, RISING);
}

void encL() 
{
  countL++;
}

void encR() 
{
  countR++;
}