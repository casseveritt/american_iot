#include "cmap.h"

ColorMap blueBlack;
ColorMap halloween;
ColorMap rgbcmy;
ColorMap maroonWhite;
ColorMap sparkle[2];

void init_cmaps() {
  blueBlack.addColor(BLUE * 0.25f);
  blueBlack.addColor(BLUE * 0.25f);
  blueBlack.addColor(CYAN);
  blueBlack.addColor(BLUE * 0.25f);
  blueBlack.addColor(BLUE * 0.25f);
  blueBlack.addColor(BLACK);
  blueBlack.addColor(BLACK);
  blueBlack.addColor(BLACK);
  blueBlack.addColor(BLACK);
  blueBlack.addColor(BLACK);

  auto orange = ORANGE * 0.0625f;
  orange.y *= 0.5f;
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(PURPLE * 0.08f);
  halloween.addColor(orange);
  halloween.addColor(orange);
  halloween.addColor(orange);
  halloween.addColor(PURPLE * 0.08f);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);
  halloween.addColor(BLACK);

  rgbcmy.addColor(RED * 0.25f);
  rgbcmy.addColor(RED * 0.25f);
  rgbcmy.addColor(RED * 0.25f);
  rgbcmy.addColor(RED * 0.25f);
  rgbcmy.addColor(GREEN * 0.25f);
  rgbcmy.addColor(GREEN * 0.25f);
  rgbcmy.addColor(GREEN * 0.25f);
  rgbcmy.addColor(GREEN * 0.25f);
  rgbcmy.addColor(BLUE * 0.25f);
  rgbcmy.addColor(BLUE * 0.25f);
  rgbcmy.addColor(BLUE * 0.25f);
  rgbcmy.addColor(BLUE * 0.25f);
  rgbcmy.addColor(CYAN * 0.25f);
  rgbcmy.addColor(CYAN * 0.25f);
  rgbcmy.addColor(CYAN * 0.25f);
  rgbcmy.addColor(CYAN * 0.25f);
  rgbcmy.addColor(MAGENTA * 0.25f);
  rgbcmy.addColor(MAGENTA * 0.25f);
  rgbcmy.addColor(MAGENTA * 0.25f);
  rgbcmy.addColor(MAGENTA * 0.25f);
  rgbcmy.addColor(YELLOW * 0.25f);
  rgbcmy.addColor(YELLOW * 0.25f);
  rgbcmy.addColor(YELLOW * 0.25f);
  rgbcmy.addColor(YELLOW * 0.25f);

  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(WHITE * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);
  maroonWhite.addColor(MAROON * 0.0625);

  sparkle[0].clear();
  sparkle[0].addColor(WHITE);
  sparkle[0].addColor(YELLOW * 0.0625f);
  sparkle[0].addColor(orange);
  sparkle[0].addColor(RED * 0.03125f);
  sparkle[0].addColor(BLACK);

  sparkle[1].clear();
  sparkle[1].addColor(WHITE);
  sparkle[1].addColor(CYAN * 0.0625f);
  sparkle[1].addColor(BLUE_VIOLET * 0.0625f);
  sparkle[1].addColor(BLUE * 0.0625f);
  sparkle[1].addColor(BLACK);
}
