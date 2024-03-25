#pragma once

enum state
{
  WAIT = 0,
  MOVING,
  ROTATION_RIGHT,
  ROTATION_LEFT,
  GIVING,
  STANDING,
  DETOUR,
  SAVECELL,
  BRICK
};