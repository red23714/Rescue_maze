#pragma once

enum state
{
  WAIT = 0,
  MOVING, //1
  ROTATION_RIGHT, //2
  ROTATION_LEFT, //3
  GIVING, //4
  STANDING, //5
  DETOUR, //6
  SAVECELL //7
};