byte sign(float value)
{
  if(value == 0) return 0;
  return value / abs(value);
}