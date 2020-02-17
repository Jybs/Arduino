extern int magicNumber;

#define CHAR_WIDTH 4
// ZERO
/*  ## 
 * #  #
 * #  #
 * #  #
 * #  #
 * #  #
 *  ##
 *     
*/
// const pZero* = [ 0b01111100, 0b10000010, 0b10000010, 0b01111100, 0x00 ];
char pZero[CHAR_WIDTH] = { 0x7C, 0x82, 0x82, 0x7C };

// ONE
/*   #  
 *  ## 
 *   # 
 *   # 
 *   # 
 *   # 
 * ####
 *    
*/
// const pOne* = [ 0b01000010, 0b11111110, 0b00000010, 0x00 ];
char pOne[CHAR_WIDTH] = { 0x02, 0x42, 0xFE, 0x02 };

// TWO
/*  ##   
 * #  # 
 *    #
 *   #
 *  # 
 * #
 * ####
 *    
*/
// const pTwo* = [ 0b01000110, 0b10001010, 0b10010010, 0b01100010, 0x00 ];
char pTwo[CHAR_WIDTH] = { 0x46, 0x8A, 0x92, 0x72 };

// THREE
/* ###   
 *    # 
 *    #
 *  ##
 *    #
 *    #
 * ### 
 *    
*/
// const pThree* = [ 0b10000010, 0b10010010, 0b10010010, 0b01101100, 0x00 ];
char pThree[CHAR_WIDTH] = { 0x82, 0x92, 0x92, 0x6C };

// FOUR
/*   #   
 * # #  
 * # # 
 * ####
 *   # 
 *   # 
 *   # 
 *    
*/
// const pFour* = [ 0b01110000, 0b00010000, 0b11111110, 0b00010000, 0x00 ];
char pFour[CHAR_WIDTH] = { 0x70, 0x10, 0xFE, 0x10 };

// FIVE
/* ###
 * # 
 * #
 *  ##
 *    #
 *    #
 * ###
 * 
*/
// const pFive* = [ 0b11100010, 0b10010010, 0b10010010, 0b00001100, 0x00 ];
char pFive[CHAR_WIDTH] = { 0xE2, 0x92, 0x92, 0x0C };

// SIX
/*   #
 *  # 
 * #
 * ###
 * #  #
 * #  #
 *  ##
 * 
*/
// const pSix* = [ 0b00111100, 0b01010010, 0b10010010, 0b00001100, 0x00 ];
char pSix[CHAR_WIDTH] = { 0x3C, 0x52, 0x92, 0x0C };

// SEVEN
/* ####
 *    #
 *   #
 *  #
 *  #
 * #
 * #
 * 
*/
// const pSeven* = [ 0b10000110, 0b10011000, 0b10100000, 0b11000000, 0x00 ];
char pSeven[CHAR_WIDTH] = { 0x86, 0x98, 0xA0, 0xC0 };

// Eight
/*  ##
 * #  #
 * #  #
 *  ##
 * #  #
 * #  #
 *  ##
 * 
*/
// const pEight* = [ 0b01101100, 0b10010010, 0b10010010, 0b01101100, 0x00 ];
char pEight[CHAR_WIDTH] = { 0x6C, 0x92, 0x92, 0x6C };

// NINE
/*  ##
 * #  #
 * #  #
 *  ###
 *    #
 *    #
 *  ##
 * 
*/
// const char* pNinet = [ 0b01100000, 0b10010010, 0b10010010, 0b01111100, 0x00 ];
char pNine[CHAR_WIDTH] = { 0x60, 0x92, 0x92, 0x7C };

// const char* pNumTable[] = { pZero, pOne, pTwo, pThree, pFour, pFive, pSix, pSeven, pEight, pNine };
char* pNumTable[] = { pZero, pOne, pTwo, pThree, pFour, pFive, pSix, pSeven, pEight, pNine };

void setChar(int offset, int position){
    CRGB color;
    char* toPrint=pNumTable[position];
    char oneColumn=0;
    if(0>position || position>9){
        toPrint=pNumTable[0];
    }
    for ( int i = 0; i < 4; i++) {
        oneColumn=*(char*)((int)toPrint+i);
        if((i+offset)&0x01){ //LED are in S, the order look like 123 654 789, depending on the index, we switch the order.
            for (int j=0, k=0; j<NUM_ROW; j++){
                if((char)0x01&(oneColumn>>j))
                    color=solidColor;
                else
                    color=backSolidColor;
                leds[(i+offset)*NUM_ROW+k++]= color;
            }
        }else{
            for (int j=NUM_ROW-1, k=0; j; j--){
                if((char)0x01&(oneColumn>>j))
                    color=solidColor;
                else
                    color=backSolidColor;
                leds[(i+offset)*NUM_ROW+k++]= color;
            }
        }
    }
}

void timerprint()
{
    static char permutt;
    int offset;
    int Sec, Min, uSec, dSec, uMin, dMin;
    int localSec=globalSec;
    Min=localSec/60;
    Sec=localSec%60;
    dMin=Min/10;
    uMin=Min%10;
    dSec=Sec/10;
    uSec=Sec%10;
    
    permutt^=1;
    
    if(dMin>9 || uMin>9 || dSec>9 || uSec>9 || dMin<0 || uMin<0 || dSec<0 || uSec<0){
        Serial.printf("dMin %u \t uMin %u \t dSec %u \t uSec %u ", dMin, uMin, dSec, uSec);
        Serial.println(""); 
        dMin=uMin=dSec=uSec=0;
    }
    
    int index=0;
    //Clean :
    for ( int i = 0; i < NUM_COL; i++) {
        for (int j=0; j<NUM_ROW; j++){
            leds[i*NUM_ROW+j]= backSolidColor;
        }
    }

    
    //Set First digit : the tens of minute
    offset=5;
    setChar(offset, dMin);

    //Set second digit : the unity of minute
    offset=10;
    setChar(offset, uMin);

    //Minutes Seconds separator
    offset=15;
    if(permutt){ //Blink each seconds
        leds[(offset)*NUM_ROW+2]= backSolidColor ;
        leds[(offset)*NUM_ROW+6]= backSolidColor ;
        leds[(offset)*NUM_ROW+3]= backSolidColor ;
        leds[(offset)*NUM_ROW+5]= backSolidColor ;
    }else{
        leds[(offset)*NUM_ROW+2]= solidColor ;
        leds[(offset)*NUM_ROW+6]= solidColor ;
        leds[(offset)*NUM_ROW+3]= solidColor ;
        leds[(offset)*NUM_ROW+5]= solidColor ;
    }
    offset=16;
    if(permutt){ //Blink each seconds
        leds[(offset)*NUM_ROW+1]= backSolidColor ;
        leds[(offset)*NUM_ROW+5]= backSolidColor ;
        leds[(offset)*NUM_ROW+2]= backSolidColor ;
        leds[(offset)*NUM_ROW+4]= backSolidColor ;
    }else{
        leds[(offset)*NUM_ROW+1]= solidColor ;
        leds[(offset)*NUM_ROW+5]= solidColor ;
        leds[(offset)*NUM_ROW+2]= solidColor ;
        leds[(offset)*NUM_ROW+4]= solidColor ;
    }
    
    //Set third digit : The tens of seconds
    offset=18;
    setChar(offset, dSec);

//     //Set fourth digit : The unity of seconds
    offset=23;
    setChar(offset, uSec);
}

void globes()
{
    for(int i=0; i<NUM_LEDS>>1; i++){
        leds[i]=solidColorFlugplatz ;
    }
    for(int i=NUM_LEDS>>1; i<NUM_LEDS; i++){
        leds[i]=solidColorMensa ;
    }
}

//This pattern is either called as a pattern or aside the timer pattern.
void stroboskop()
{
    CRGB permutSolidColor;
#ifndef LEDSTRIP_MATRICE
    //blink in two colors, fill always with only one of both colors.
    static char permutt;
    permutt^=1;
    if(permutt){
        //foreground
        fill_solid(leds, NUM_LEDS, solidColor);
    }else{
        //background
        fill_solid(leds, NUM_LEDS, backSolidColor);
    }
#endif
#ifdef LEDSTRIP_MATRICE
    permutSolidColor=solidColor;
    solidColor=backSolidColor;
    backSolidColor=permutSolidColor;
#endif
}

void showSolidColor()
{
  fill_solid(leds, NUM_LEDS, solidColor);
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 10);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}



void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS - 1);
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for ( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle()
{
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++)
  {
    leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

// Pride2015 by Mark Kriegsman: https://gist.github.com/kriegsman/964de772d64c502760e5
// This function draws rainbows with an ever-changing,
// widely-varying set of parameters.
void pride() {
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for ( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV( hue8, sat8, bri8);

    nblend(leds[i], newcolor, 64);
  }
}


// ColorWavesWithPalettes by Mark Kriegsman: https://gist.github.com/kriegsman/8281905786e8b2632aeb
// This function draws color waves with an ever-changing,
// widely-varying set of parameters, using a color palette.
void colorwaves()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  // uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 300, 1500);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for ( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
    uint16_t h16_128 = hue16 >> 7;
    if ( h16_128 & 0x100) {
      hue8 = 255 - (h16_128 >> 1);
    } else {
      hue8 = h16_128 >> 1;
    }

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    uint8_t index = hue8;
    //index = triwave8( index);
    index = scale8( index, 240);

    CRGB newcolor = ColorFromPalette(gCurrentPalette, index, bri8);

    nblend(leds[i], newcolor, 128);
  }
}


// adapted from ColorWavesWithPalettes by Mark Kriegsman: https://gist.github.com/kriegsman/8281905786e8b2632aeb
// This function draws color waves with an ever-changing,
// widely-varying set of parameters, using a color palette.
void fire()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  // uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 300, 1500);
      
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for ( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    if (random(0,255) > 200) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
    uint16_t h16_128 = hue16 >> 7;
    if ( h16_128 & 0x100) {
      hue8 = 255 - (h16_128 >> 1);
    } else {
      hue8 = h16_128 >> 1;
    }

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    uint8_t index = random(0,255);
    //index = triwave8( index);
    index = scale8( index, 240);
    gCurrentPalette = gp_fire;
    CRGB newcolor = ColorFromPalette(gCurrentPalette, index, bri8);        
    nblend(leds[i], newcolor, 128);
    }
   }
 }
       
// Alternate rendering function just scrolls the current palette
// across the defined LED strip.
void palettetest()
{
  static uint8_t startindex = 0;
  startindex--;
  fill_palette( leds, NUM_LEDS, startindex, (256 / NUM_LEDS) + 1,
                gCurrentPalette, 255, LINEARBLEND);
}
