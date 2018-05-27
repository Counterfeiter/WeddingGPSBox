#include <Wire.h>
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include <Servo.h>
#include "src/TinyGPSPlus_1.0.2/src/TinyGPS++.h"
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <math.h>

#define TESTMODE_BANDY
/*
 * 51.340293, 12.331978
 * 51.342043, 12.330518
 * 51.342713, 12.331173
 * 51.343082, 12.332589
 * 51.341155, 12.335382
 * 51.339222, 12.334542
 */
//#define TESTMODE_MARATZI

//if there is not enough SRAM you could select this option to put all waypoints 
//to the progmem
//#define USE_PROGMEM_FOR_LOCATIONS

#define LCD_DISPLAY_CHARLENGTH    20
#define LCD_DISPLAY_LINES         4

#define GPS_WAYPOINTTOLERNACE     150 //m
#define GPS_NUM_OF_WAYPOINTS      4

#define GPS_MIN_HDOP              300.0

//define byte index in eeprom -> we have enough storage here
#define EEPROM_DATE_TIME_POS      0
#define EEPROM_CURRENT_QUEST_POS  30

#define WAITTIME_FOR_DISP_TEXT    5000 //ms

#if defined(TESTMODE_BANDY)
  #define WAITTIME_FOR_NEXT_POS   (1 * 60)
#else
  #define WAITTIME_FOR_NEXT_POS   (5 * 60) //5 minutes
#endif

#define SERVO_PIN                 7
#define SERVO_ENGAGE              20 //deg
#define SERVO_DISENGAGE           160 //deg

LiquidCrystal_I2C   lcd(0x27, LCD_DISPLAY_CHARLENGTH, LCD_DISPLAY_LINES);
Servo               lock;
TinyGPSPlus         gps;

struct date_s {
  int Year;
  int Month;
  int Date;
};

struct time_s {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
};

struct datetime {
  struct date_s date;
  struct time_s time;
};


struct date_s past;
struct date_s timeElapsed;

struct Positions_s
{
  float position_lat;
  float position_lng;
  char name_of_position[LCD_DISPLAY_CHARLENGTH + 1];
  char textBefore[LCD_DISPLAY_LINES][LCD_DISPLAY_CHARLENGTH + 1];
  char textAfter[LCD_DISPLAY_LINES][LCD_DISPLAY_CHARLENGTH + 1];
};

//put this little data into the sram (3x2 Byte)
const struct date_s weddingdata
{
#if defined(TESTMODE_BANDY)
  2016,
  7,
  1
#else
  2018,
  6,
  2
#endif
};

#if defined(USE_PROGMEM_FOR_LOCATIONS)
const struct Positions_s positions[GPS_NUM_OF_WAYPOINTS] PROGMEM =
#else
const struct Positions_s positions[GPS_NUM_OF_WAYPOINTS] =
#endif
{
    //johannapark
  {
#if defined(TESTMODE_BANDY)
    51.340293, 
    12.331978,
#elif defined(TESTMODE_MARATZI)
    51.389551, 
    12.162822,
#else
    51.334754, 
    12.364144,
#endif
    "Johannapark", //name
    { "Gealtert seid ihr",
    "hier, jedoch nicht",
    "vor eurer Tuer.",
    "",
    },
    {
    "Die Besten Feste!",
    "Direkt hier!",
    "Koennt ihr euch",
    "erinnern?",
    },
  },
  //stroemthaler see
  {
#if defined(TESTMODE_BANDY)
    51.342043, 
    12.330518,
#elif defined(TESTMODE_MARATZI)
    51.389101, 
    12.162841,
#else
    51.228377, 
    12.439552,
#endif
    "Highfield", //name
    { "Wasser und Sonne,",
    "welch eine Wonne!",
    "Doch so romantisch,",
    "ists ganz & gar nich",
    },
    {
    "Campen, Musik und",
    "Katzenwaesche...",
    "Fuehlt mal den Bass",
    "...",
    },
  },
  //Garten
  {
#if defined(TESTMODE_BANDY)
    51.342713, 
    12.331173,
#elif defined(TESTMODE_MARATZI)
    51.388561, 
    12.164328,
#else
    51.345421,
    12.325850,
#endif
    "Garten",
    {
      "Schoen ist´s hier,",
      "mit Roster und Bier!",
      "Erholung pur!",
      "Wo sind wir hier nur?",
    },
    {
      "Haengematte Ahoi!",
      "Jetzt koennt ihr",
      "erst einmal entspan-",
      "nen.",
    },
  },
  //Wohnung Jami
  {
#if defined(TESTMODE_BANDY)
    51.343082, 
    12.332589,
#elif defined(TESTMODE_MARATZI)
    51.388079, 
    12.165237,
#else
    51.346405, 
    12.319637,
#endif
    "Wohnung",
    {
      "Der schoenste Ort,",
      "ungern will man fort!",
      "Alles habt ihr hier,",
      "gar ein haarig Tier",
    },
    {
      "Ihr habt es endlich",
      "geschafft! Die Box",
      "oeffnet sich beim",
      "naechsten Versuch!",
    }
  }
};




char *P(const char *stringPROGMEM)
{
#if defined(USE_PROGMEM_FOR_LOCATIONS)
  static char stringbuf[LCD_DISPLAY_CHARLENGTH + 1] = { '\0' };

  memset(stringbuf, (int)' ', sizeof(stringbuf - 1));

  strcpy_P(stringbuf, stringPROGMEM);

  return stringbuf;
#else
  return stringPROGMEM;
#endif
}

int getNoOfDays(int Month, int Year)
{
  const uint8_t noOfDays[] =
  { 0, 31, 0, 31, 30, 31, 30, 31,
    31, 30, 31, 30, 31
  };
  
  if(Month >= sizeof(noOfDays)) Month = 1;
  
  if (Month == 2) {
    if (Year % 4 == 0 || Year % 400 == 0)
      return 29;
    else if (Year % 4 != 0 || Year % 100 == 0)
      return 28;
  }
  return noOfDays[Month];
}

uint8_t timeSince(const date_s *current, const date_s *past, date_s *timeElapsed_out)
{
  //Calculate years passed
  timeElapsed_out->Year = current->Year - past->Year;

  //Calculate the months passed and correct the number of years if required
  if (current->Month < past->Month) 
  {
    timeElapsed_out->Year -= 1;
    timeElapsed_out->Month = 12 - (abs(current->Month - past->Month));
  }
  else
    timeElapsed_out->Month = (current->Month - past->Month);

  //Calculate the days passed and correct the number of months
  if (current->Date < past->Date) 
  {
    timeElapsed_out->Month -= 1;
    timeElapsed_out->Date = (getNoOfDays(current->Month, current->Year)) - (abs(current->Date - past->Date));
  }
  else
    timeElapsed_out->Date = (current->Date - past->Date);


  return 1;
}

void GPSReadData(HardwareSerial& s)
{
  while (s.available())
  {
    char c = s.read();
    gps.encode(c);
    //Serial.write(c); //debug
  }
}

void GPSNonBlockingDelay(unsigned long ms, HardwareSerial& s)
{
  unsigned long start = millis();
  do 
  {
    while (s.available())
    {
      GPSReadData(s);
    }
  } while (millis() - start < ms);
}

unsigned long crc32(const char *data, int len) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < len  ; ++index) {
    crc = crc_table[(crc ^ data[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (data[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

int readFromEEPROM(char *data, int len, int index)
{
  Serial.println(F("Read eeprom"));
  
  for(int i = 0; i < len; i++)
  {
    data[i] = EEPROM.read(index);
    index++;
  }

  unsigned long crc_ee;
  char *crc_p = (char*)&crc_ee;
  unsigned long crc = crc32(data, len);

  *crc_p = EEPROM.read(index);
  index++; crc_p++;
  *crc_p = EEPROM.read(index);
  index++; crc_p++;
  *crc_p = EEPROM.read(index);
  index++; crc_p++;
  *crc_p = EEPROM.read(index);

  return (crc == crc_ee) ? 1 : 0;
}

void writeToEEPROM(const char *data, int len, int index)
{
  Serial.println(F("Write eeprom"));
  unsigned long crc = crc32(data, len);
  char *crc_p = (char*)&crc;
  for(int i = 0; i < len; i++)
  {
    if(EEPROM.read(index) != data[i])  
      EEPROM.write(index, data[i]);
    index++;
  }
  for(int i = index; i < (index + 4); i++)
  {
    if(EEPROM.read(index) != *crc_p)  
      EEPROM.write(i, *crc_p);
    crc_p++;
  }
}

void setup() {
  lcd.begin();
  lock.write(SERVO_ENGAGE);
  lock.attach(SERVO_PIN);
  lock.write(SERVO_ENGAGE);

  Serial.begin(115200);
  Serial2.begin(9600);

  pinMode(13, INPUT_PULLUP); //reset pin on mega port directly near GND
  delay(50); //wait to settle high level
  if(LOW == digitalRead(13))
  {
    //reset pos
    uint8_t reset_pos = 0;
    writeToEEPROM((char*)&reset_pos, sizeof(reset_pos), 30);

    while(1);
  }
}

void loop() {
  static uint8_t coordinats_shown = 0;
  //read gps data
  GPSReadData(Serial2);

  uint8_t act_quest = 0;


  if(readFromEEPROM((char*)&act_quest, sizeof(act_quest), 30))
  {
    if(act_quest >= GPS_NUM_OF_WAYPOINTS)
    {
      //display gelöst ....
      //set servo to open
      lock.write(SERVO_DISENGAGE);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Ihr hast die Aufgabe"));
      lcd.setCursor(0,1);
      lcd.print(F("geloest. Viel Spass "));
      lcd.setCursor(0,2);
      lcd.print(F("mit eurem Geschenk. "));
      lcd.setCursor(0,3);
      lcd.print(F("Bandy & Maratzi     "));
      GPSNonBlockingDelay(10000, Serial2);
      return;
    }
  }
  else
  {
    //start from beginning if eeprom is empty or corrupt
    act_quest = 0;
  }

  //gps module outputs time, but if time not valid it starts with 2006
  if( !(gps.time.isValid() && gps.date.isValid() && gps.date.year() != 2006) )
  {
    //loop until time is ready
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Warte auf GPS-Zeit.."));
    GPSNonBlockingDelay(2000, Serial2);
    return;
  }
  datetime dt = 
  {
    {gps.date.year(), gps.date.month(), gps.date.day()}, 
    {gps.time.hour(), gps.time.minute(), gps.time.second()}
  };

  date_s diff;

  if(timeSince(&dt.date, &weddingdata, &diff))
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Verheiratet seit:   "));
    lcd.setCursor(0,1);
    lcd.print(diff.Year, DEC);
    lcd.print(F(" Jahr(en)"));
    lcd.setCursor(0,2);
    lcd.print(diff.Month, DEC);
    lcd.print(F(" Monat(en)"));
    lcd.setCursor(0,3);
    lcd.print(diff.Date, DEC);
    lcd.print(F(" Tag(en)"));

    GPSNonBlockingDelay(WAITTIME_FOR_DISP_TEXT, Serial2);
  }

  
  lcd.clear();
  for(int i = 0; i < LCD_DISPLAY_LINES; i++)
  {
    lcd.setCursor(0,i);
    lcd.print(P(positions[act_quest].textBefore[i]));
  }
  
  GPSNonBlockingDelay(WAITTIME_FOR_DISP_TEXT, Serial2);

  datetime dt_ee; 
  if(readFromEEPROM((char*)&dt_ee, sizeof(dt_ee), 0) && !coordinats_shown)
  {
    Serial.println("Zeitangabe aus EEPROM geladen");
    //we got a error if someone use this in noon time... but who will do this? =)
     if(dt.date.Year == dt_ee.date.Year && dt.date.Month == dt_ee.date.Month && dt.date.Date == dt_ee.date.Date)
     {
      int32_t sec = (int32_t)dt.time.hour * 3600 + (int32_t)dt.time.min * 60 + dt.time.sec;
      sec -= ((int32_t)dt_ee.time.hour * 3600 + (int32_t)dt_ee.time.min * 60 + dt_ee.time.sec);

      Serial.println(sec);

      if(sec > 0 && sec < WAITTIME_FOR_NEXT_POS)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("Entfernungsabfrage  "));
        lcd.setCursor(0,1);
        lcd.print(F("fuer die naechsten  "));
        lcd.setCursor(0,2);
        lcd.print(F("            Sekunden"));
        lcd.setCursor(0,2);
        lcd.print(WAITTIME_FOR_NEXT_POS - sec, DEC);
        lcd.setCursor(0,3);
        lcd.print(F("blockiert...        "));
        GPSNonBlockingDelay(WAITTIME_FOR_DISP_TEXT, Serial2);
        return;
      }
     }
  }

  if(gps.location.isUpdated() && gps.hdop.isValid() && (int)gps.hdop.value() != 0 && (double)gps.hdop.value() < GPS_MIN_HDOP)
  {
    int32_t hdop = gps.hdop.value() * 100.0;
    static uint8_t km = 0;
    static double distance;
    if(!coordinats_shown)
    {
      #if defined(USE_PROGMEM_FOR_LOCATIONS)
      double lat = (double)pgm_read_float_near(&positions[act_quest].position_lat);
      double lng = (double)pgm_read_float_near(&positions[act_quest].position_lng);
      #else
      double lat = positions[act_quest].position_lat;
      double lng = positions[act_quest].position_lng;
      #endif
  
      
      distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), lat, lng);

      if(distance < GPS_WAYPOINTTOLERNACE)
      {
        act_quest++;
        writeToEEPROM((char*)&act_quest, sizeof(act_quest), 30);

        while(1)
        {
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(F("Wegpunkt:"));
            lcd.setCursor(0,1);
            lcd.print(P(positions[act_quest - 1].name_of_position));
            lcd.setCursor(0,2);
            lcd.print(F("erreicht!"));
            
            GPSNonBlockingDelay(WAITTIME_FOR_DISP_TEXT, Serial2);
            
            lcd.clear();
            for(int i = 0; i < LCD_DISPLAY_LINES; i++)
            {
              lcd.setCursor(0,i);
              lcd.print(P(positions[act_quest - 1].textAfter[i]));
            }
  
            GPSNonBlockingDelay(WAITTIME_FOR_DISP_TEXT, Serial2);
        }
      }
      else
      {
  
        if(distance >= 1000.0)
        {
          distance *= 0.001; //set to km
          km = 1;
        }
        
        writeToEEPROM((const char*)&dt, sizeof(dt), 0);
      }
    }

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Distanz zum Ziel:"));
    lcd.setCursor(0,1);
    if(km)
    {
      lcd.print(distance, 1);
      lcd.print(F(" km"));
    }
    else
    {
      lcd.print(distance, 0);
      lcd.print(F(" m"));
    }

    lcd.setCursor(0, 2);
    lcd.print(hdop, DEC);

    coordinats_shown = 1;

    GPSNonBlockingDelay(WAITTIME_FOR_DISP_TEXT, Serial2);
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Warte auf           "));
    lcd.setCursor(0,1);
    lcd.print(F("GPS-Position...     "));
    if(gps.hdop.isValid() && (int)gps.hdop.value() != 0 )
    {
      lcd.setCursor(0,2);
      lcd.print((double)gps.hdop.value(), 0);
      lcd.print(" < ");
      lcd.print(GPS_MIN_HDOP, 0);
      lcd.print(" ?");
    }
    GPSNonBlockingDelay(2000, Serial2);
  }
}
