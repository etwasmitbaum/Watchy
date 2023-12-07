#include "Watchy.h"
#include "bma456w.h"

WatchyRTC Watchy::RTC;
GxEPD2_BW<WatchyDisplay, WatchyDisplay::HEIGHT> Watchy::display(
    WatchyDisplay(DISPLAY_CS, DISPLAY_DC, DISPLAY_RES, DISPLAY_BUSY));

RTC_DATA_ATTR int guiState;
RTC_DATA_ATTR int menuIndex;
//RTC_DATA_ATTR BMA423 sensor;
RTC_DATA_ATTR bool WIFI_CONFIGURED;
RTC_DATA_ATTR bool BLE_CONFIGURED;
RTC_DATA_ATTR weatherData currentWeather;
RTC_DATA_ATTR int weatherIntervalCounter = -1;
RTC_DATA_ATTR bool displayFullInit       = true;
RTC_DATA_ATTR long gmtOffset = 0;
RTC_DATA_ATTR bool alreadyInMenu         = true;
RTC_DATA_ATTR tmElements_t bootTime;

static uint8_t dev_addr;

bool test_accell = false;

static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);
int8_t bmi4xx_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bma4xx_hal_delay_usec(uint32_t period_us, void *intf_ptr);
int8_t bma4xx_hal_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bmi4xx_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t BMA456_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);
int8_t BMA456_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);
int8_t bma4_interface_selection(struct bma4_dev *bma);


/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH      (9.80665f)

/*! Macro that holds the total number of accel x,y and z axes sample counts to be printed */
#define ACCEL_SAMPLE_COUNT UINT8_C(100)

/* Variable to store the status of API */
int8_t rslt;

/* Sensor initialization configuration */
struct bma4_dev bma = { 0 };

/* Variable to store accel data ready interrupt status */
uint16_t int_status = 0;

static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
  float half_scale = ((float)(1 << bit_width) / 2.0f);

  return (GRAVITY_EARTH * val * g_range) / half_scale;
}

struct bma4_accel sens_data = { 0 };
float x = 0, y = 0, z = 0;

void Watchy::init(String datetime) {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause(); // get wake up reason
  Wire.begin(SDA, SCL);                         // init i2c
  RTC.init();

  // Init the display here for all cases, if unused, it will do nothing
  display.epd2.selectSPI(SPI, SPISettings(20000000, MSBFIRST, SPI_MODE0)); // Set SPI to 20Mhz (default is 4Mhz)
  display.init(0, displayFullInit, 10,
               true); // 10ms by spec, and fast pulldown reset
  display.epd2.setBusyCallback(displayBusyCallback);

  Serial.begin(115200);
  Serial.println("Start");

//  delay(5000);
//
//    /* Variable that holds the accelerometer sample count */
//  uint8_t n_data = ACCEL_SAMPLE_COUNT;
//
//  struct bma4_accel_config accel_conf = { 0 };
//
//  /* Function to select interface between SPI and I2C, according to that the device structure gets updated */
//  rslt = bma4_interface_selection(&bma);
//  if (rslt != BMA4_OK)
//  {
//    Serial.print("bma4_interface_selection Error -");
//    Serial.println(rslt);
//  }
//
//  /* Sensor initialization */
//  rslt = bma456w_init(&bma);
//  if (rslt != BMA4_OK)
//  {
//    Serial.print("bma456_init status Error -");
//    Serial.println(rslt);
//  }
//  /* Upload the configuration file to enable the features of the sensor. */
//  rslt = bma456w_write_config_file(&bma);
//  if (rslt != BMA4_OK)
//  {
//    Serial.print("bma456_write_config status Error -");
//    Serial.println(rslt);
//  }
//  /* Enable the accelerometer */
//  rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
//  if (rslt != BMA4_OK)
//  {
//    Serial.print("bma4_set_accel_enable status Error -");
//    Serial.println(rslt);
//  }
//  /* Accelerometer configuration settings */
//  /* Output data Rate */
//  accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;
//
//  /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
//  accel_conf.range = BMA4_ACCEL_RANGE_2G;
//
//  /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
//     if it is set to 2, then 2^(bandwidth parameter) samples
//     are averaged, resulting in 4 averaged samples
//     Note1 : For more information, refer the datasheet.
//     Note2 : A higher number of averaged samples will result in a less noisier signal, but
//     this has an adverse effect on the power consumed.
//  */
//  accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
//
//  /* Enable the filter performance mode where averaging of samples
//     will be done based on above set bandwidth and ODR.
//     There are two modes
//      0 -> Averaging samples (Default)
//      1 -> No averaging
//     For more info on No Averaging mode refer datasheet.
//  */
//  accel_conf.perf_mode = BMA4_CIC_AVG_MODE;
//
//  /* Set the accel configurations */
//  rslt = bma4_set_accel_config(&accel_conf, &bma);
//
//  if (rslt != BMA4_OK)
//  {
//    Serial.print("bma4_set_accel_config status Error -");
//    Serial.println(rslt);
//  }
//  /* Mapping data ready interrupt with interrupt pin 1 to get interrupt status once getting new accel data */
//  rslt = bma456w_map_interrupt(BMA4_INTR1_MAP, BMA4_DATA_RDY_INT, BMA4_ENABLE, &bma);
//  if (rslt != BMA4_OK)
//  {
//    Serial.print("bma456_map_interrupt status Error -");
//    Serial.println(rslt);
//  }
//  Serial.println("Ax[m/s2], Ay[m/s2], Az[m/s2]\r\n");
//
//  while (test_accell)
//  {
//      /* Read interrupt status */
//  rslt = bma456w_read_int_status(&int_status, &bma);
//  if (rslt != BMA4_OK)
//  {
//    Serial.print("bma456_read_int_status Error -");
//    Serial.println(rslt);
//  }
//
//  /* Filtering only the accel data ready interrupt */
//  if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
//  {
//    /* Read the accel x, y, z data */
//    rslt = bma4_read_accel_xyz(&sens_data, &bma);
//
//    if (rslt == BMA4_OK)
//    {
//      /* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
//      x = lsb_to_ms2(sens_data.x, 2, bma.resolution);
//      y = lsb_to_ms2(sens_data.y, 2, bma.resolution);
//      z = lsb_to_ms2(sens_data.z, 2, bma.resolution);
//
//      /* Print the data in m/s2 */
//      Serial.print("X = ");
//      Serial.print(x);
//      Serial.print("\t");
//      Serial.print("Y = ");
//      Serial.print(y);
//      Serial.print("\t");
//      Serial.print("Z = ");
//      Serial.println(z);
//    }
//  }
//
//  }
  

  switch (wakeup_reason) {
  case ESP_SLEEP_WAKEUP_EXT0: // RTC Alarm
    RTC.read(currentTime);
    switch (guiState) {
    case WATCHFACE_STATE:
      showWatchFace(true); // partial updates on tick
      if (settings.vibrateOClock) {
        if (currentTime.Minute == 0) {
          // The RTC wakes us up once per minute
          vibMotor(75, 4);
        }
      }
      break;
    case MAIN_MENU_STATE:
      // Return to watchface if in menu for more than one tick
      if (alreadyInMenu) {
        guiState = WATCHFACE_STATE;
        showWatchFace(false);
      } else {
        alreadyInMenu = true;
      }
      break;
    }
    break;
  case ESP_SLEEP_WAKEUP_EXT1: // button Press
    handleButtonPress();
    break;
  default: // reset
    RTC.config(datetime);
    //_bmaConfig();
    gmtOffset = settings.gmtOffset;
    RTC.read(currentTime);
    RTC.read(bootTime);
    showWatchFace(false); // full update on reset
    vibMotor(75, 4);
    break;
  }
  deepSleep();
}

void Watchy::displayBusyCallback(const void *) {
  gpio_wakeup_enable((gpio_num_t)DISPLAY_BUSY, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
}

void Watchy::deepSleep() {
  display.hibernate();
  if (displayFullInit) // For some reason, seems to be enabled on first boot
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  displayFullInit = false; // Notify not to init it again
  RTC.clearAlarm();        // resets the alarm flag in the RTC

  // Set GPIOs 0-39 to input to avoid power leaking out
  const uint64_t ignore = 0b11110001000000110000100111000010; // Ignore some GPIOs due to resets
  for (int i = 0; i < GPIO_NUM_MAX; i++) {
    if ((ignore >> i) & 0b1)
      continue;
    pinMode(i, INPUT);
  }
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RTC_INT_PIN,
                               0); // enable deep sleep wake on RTC interrupt
  esp_sleep_enable_ext1_wakeup(
      BTN_PIN_MASK,
      ESP_EXT1_WAKEUP_ANY_HIGH); // enable deep sleep wake on button press
  esp_deep_sleep_start();
}

void Watchy::handleButtonPress() {
  uint64_t wakeupBit = esp_sleep_get_ext1_wakeup_status();
  // Menu Button
  if (wakeupBit & MENU_BTN_MASK) {
    if (guiState ==
        WATCHFACE_STATE) { // enter menu state if coming from watch face
      showMenu(menuIndex, false);
    } else if (guiState ==
               MAIN_MENU_STATE) { // if already in menu, then select menu item
      switch (menuIndex) {
      case 0:
        showAbout();
        break;
      case 1:
        showBuzz();
        break;
      case 2:
        showAccelerometer();
        break;
      case 3:
        setTime();
        break;
      case 4:
        setupWifi();
        break;
      case 5:
        showUpdateFW();
        break;
      case 6:
        showSyncNTP();
        break;
      default:
        break;
      }
    } else if (guiState == FW_UPDATE_STATE) {
      updateFWBegin();
    }
  }
  // Back Button
  else if (wakeupBit & BACK_BTN_MASK) {
    if (guiState == MAIN_MENU_STATE) { // exit to watch face if already in menu
      RTC.read(currentTime);
      showWatchFace(false);
    } else if (guiState == APP_STATE) {
      showMenu(menuIndex, false); // exit to menu if already in app
    } else if (guiState == FW_UPDATE_STATE) {
      showMenu(menuIndex, false); // exit to menu if already in app
    } else if (guiState == WATCHFACE_STATE) {
      return;
    }
  }
  // Up Button
  else if (wakeupBit & UP_BTN_MASK) {
    if (guiState == MAIN_MENU_STATE) { // increment menu index
      menuIndex--;
      if (menuIndex < 0) {
        menuIndex = MENU_LENGTH - 1;
      }
      showMenu(menuIndex, true);
    } else if (guiState == WATCHFACE_STATE) {
      return;
    }
  }
  // Down Button
  else if (wakeupBit & DOWN_BTN_MASK) {
    if (guiState == MAIN_MENU_STATE) { // decrement menu index
      menuIndex++;
      if (menuIndex > MENU_LENGTH - 1) {
        menuIndex = 0;
      }
      showMenu(menuIndex, true);
    } else if (guiState == WATCHFACE_STATE) {
      return;
    }
  }

  /***************** fast menu *****************/
  bool timeout     = false;
  long lastTimeout = millis();
  pinMode(MENU_BTN_PIN, INPUT);
  pinMode(BACK_BTN_PIN, INPUT);
  pinMode(UP_BTN_PIN, INPUT);
  pinMode(DOWN_BTN_PIN, INPUT);
  while (!timeout) {
    if (millis() - lastTimeout > 5000) {
      timeout = true;
    } else {
      if (digitalRead(MENU_BTN_PIN) == 1) {
        lastTimeout = millis();
        if (guiState ==
            MAIN_MENU_STATE) { // if already in menu, then select menu item
          switch (menuIndex) {
          case 0:
            showAbout();
            break;
          case 1:
            showBuzz();
            break;
          case 2:
            showAccelerometer();
            break;
          case 3:
            setTime();
            break;
          case 4:
            setupWifi();
            break;
          case 5:
            showUpdateFW();
            break;
          case 6:
            showSyncNTP();
            break;
          default:
            break;
          }
        } else if (guiState == FW_UPDATE_STATE) {
          updateFWBegin();
        }
      } else if (digitalRead(BACK_BTN_PIN) == 1) {
        lastTimeout = millis();
        if (guiState ==
            MAIN_MENU_STATE) { // exit to watch face if already in menu
          RTC.read(currentTime);
          showWatchFace(false);
          break; // leave loop
        } else if (guiState == APP_STATE) {
          showMenu(menuIndex, false); // exit to menu if already in app
        } else if (guiState == FW_UPDATE_STATE) {
          showMenu(menuIndex, false); // exit to menu if already in app
        }
      } else if (digitalRead(UP_BTN_PIN) == 1) {
        lastTimeout = millis();
        if (guiState == MAIN_MENU_STATE) { // increment menu index
          menuIndex--;
          if (menuIndex < 0) {
            menuIndex = MENU_LENGTH - 1;
          }
          showFastMenu(menuIndex);
        }
      } else if (digitalRead(DOWN_BTN_PIN) == 1) {
        lastTimeout = millis();
        if (guiState == MAIN_MENU_STATE) { // decrement menu index
          menuIndex++;
          if (menuIndex > MENU_LENGTH - 1) {
            menuIndex = 0;
          }
          showFastMenu(menuIndex);
        }
      }
    }
  }
}

void Watchy::showMenu(byte menuIndex, bool partialRefresh) {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);

  int16_t x1, y1;
  uint16_t w, h;
  int16_t yPos;

  const char *menuItems[] = {
      "About Watchy", "Vibrate Motor", "Show Accelerometer",
      "Set Time",     "Setup WiFi",    "Update Firmware",
      "Sync NTP"};
  for (int i = 0; i < MENU_LENGTH; i++) {
    yPos = MENU_HEIGHT + (MENU_HEIGHT * i);
    display.setCursor(0, yPos);
    if (i == menuIndex) {
      display.getTextBounds(menuItems[i], 0, yPos, &x1, &y1, &w, &h);
      display.fillRect(x1 - 1, y1 - 10, 200, h + 15, GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);
      display.println(menuItems[i]);
    } else {
      display.setTextColor(GxEPD_WHITE);
      display.println(menuItems[i]);
    }
  }

  display.display(partialRefresh);

  guiState = MAIN_MENU_STATE;
  alreadyInMenu = false;
}

void Watchy::showFastMenu(byte menuIndex) {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);

  int16_t x1, y1;
  uint16_t w, h;
  int16_t yPos;

  const char *menuItems[] = {
      "About Watchy", "Vibrate Motor", "Show Accelerometer",
      "Set Time",     "Setup WiFi",    "Update Firmware",
      "Sync NTP"};
  for (int i = 0; i < MENU_LENGTH; i++) {
    yPos = MENU_HEIGHT + (MENU_HEIGHT * i);
    display.setCursor(0, yPos);
    if (i == menuIndex) {
      display.getTextBounds(menuItems[i], 0, yPos, &x1, &y1, &w, &h);
      display.fillRect(x1 - 1, y1 - 10, 200, h + 15, GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);
      display.println(menuItems[i]);
    } else {
      display.setTextColor(GxEPD_WHITE);
      display.println(menuItems[i]);
    }
  }

  display.display(true);

  guiState = MAIN_MENU_STATE;
}

void Watchy::showAbout() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 20);

  //Library Version
  display.print("LibVer: ");
  display.println(WATCHY_LIB_VER);
  //RTC Type
  const char *RTC_HW[3] = {"<UNKNOWN>", "DS3231", "PCF8563"};
  display.print("RTC: ");
  display.println(RTC_HW[RTC.rtcType]); // 0 = UNKNOWN, 1 = DS3231, 2 = PCF8563
  //Battery Level
  display.print("Batt: ");
  float voltage = getBatteryVoltage();
  display.print(voltage);
  display.println("V");
  //Uptime
  display.print("Uptime: ");
  RTC.read(currentTime);
  time_t b = makeTime(bootTime);
  time_t c = makeTime(currentTime);
  int totalSeconds = c-b;
  //int seconds = (totalSeconds % 60);
  int minutes = (totalSeconds % 3600) / 60;
  int hours = (totalSeconds % 86400) / 3600;
  int days = (totalSeconds % (86400 * 30)) / 86400;
  display.print(days);
  display.print("d");
  display.print(hours);
  display.print("h");
  display.print(minutes);
  display.println("m");
  //WiFi Info
  display.print("IP: ");
  display.println(WiFi.localIP());
	display.println("MAC Address:");
	display.println(WiFi.macAddress());

  display.display(false); // full refresh

  guiState = APP_STATE;
}

void Watchy::showBuzz() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(70, 80);
  display.println("Buzz!");
  display.display(false); // full refresh
  vibMotor();
  showMenu(menuIndex, false);
}

void Watchy::vibMotor(uint8_t intervalMs, uint8_t length) {
  pinMode(VIB_MOTOR_PIN, OUTPUT);
  bool motorOn = false;
  for (int i = 0; i < length; i++) {
    motorOn = !motorOn;
    digitalWrite(VIB_MOTOR_PIN, motorOn);
    delay(intervalMs);
  }
}

void Watchy::setTime() {

  guiState = APP_STATE;

  RTC.read(currentTime);

  int8_t minute = currentTime.Minute;
  int8_t hour   = currentTime.Hour;
  int8_t day    = currentTime.Day;
  int8_t month  = currentTime.Month;
  int8_t year   = tmYearToY2k(currentTime.Year);

  int8_t setIndex = SET_HOUR;

  int8_t blink = 0;

  pinMode(DOWN_BTN_PIN, INPUT);
  pinMode(UP_BTN_PIN, INPUT);
  pinMode(MENU_BTN_PIN, INPUT);
  pinMode(BACK_BTN_PIN, INPUT);

  display.setFullWindow();

  while (1) {

    if (digitalRead(MENU_BTN_PIN) == 1) {
      setIndex++;
      if (setIndex > SET_DAY) {
        break;
      }
    }
    if (digitalRead(BACK_BTN_PIN) == 1) {
      if (setIndex != SET_HOUR) {
        setIndex--;
      }
    }

    blink = 1 - blink;

    if (digitalRead(DOWN_BTN_PIN) == 1) {
      blink = 1;
      switch (setIndex) {
      case SET_HOUR:
        hour == 23 ? (hour = 0) : hour++;
        break;
      case SET_MINUTE:
        minute == 59 ? (minute = 0) : minute++;
        break;
      case SET_YEAR:
        year == 99 ? (year = 0) : year++;
        break;
      case SET_MONTH:
        month == 12 ? (month = 1) : month++;
        break;
      case SET_DAY:
        day == 31 ? (day = 1) : day++;
        break;
      default:
        break;
      }
    }

    if (digitalRead(UP_BTN_PIN) == 1) {
      blink = 1;
      switch (setIndex) {
      case SET_HOUR:
        hour == 0 ? (hour = 23) : hour--;
        break;
      case SET_MINUTE:
        minute == 0 ? (minute = 59) : minute--;
        break;
      case SET_YEAR:
        year == 0 ? (year = 99) : year--;
        break;
      case SET_MONTH:
        month == 1 ? (month = 12) : month--;
        break;
      case SET_DAY:
        day == 1 ? (day = 31) : day--;
        break;
      default:
        break;
      }
    }

    display.fillScreen(GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&DSEG7_Classic_Bold_53);

    display.setCursor(5, 80);
    if (setIndex == SET_HOUR) { // blink hour digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (hour < 10) {
      display.print("0");
    }
    display.print(hour);

    display.setTextColor(GxEPD_WHITE);
    display.print(":");

    display.setCursor(108, 80);
    if (setIndex == SET_MINUTE) { // blink minute digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (minute < 10) {
      display.print("0");
    }
    display.print(minute);

    display.setTextColor(GxEPD_WHITE);

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(45, 150);
    if (setIndex == SET_YEAR) { // blink minute digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    display.print(2000 + year);

    display.setTextColor(GxEPD_WHITE);
    display.print("/");

    if (setIndex == SET_MONTH) { // blink minute digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (month < 10) {
      display.print("0");
    }
    display.print(month);

    display.setTextColor(GxEPD_WHITE);
    display.print("/");

    if (setIndex == SET_DAY) { // blink minute digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (day < 10) {
      display.print("0");
    }
    display.print(day);
    display.display(true); // partial refresh
  }

  tmElements_t tm;
  tm.Month  = month;
  tm.Day    = day;
  tm.Year   = y2kYearToTm(year);
  tm.Hour   = hour;
  tm.Minute = minute;
  tm.Second = 0;

  RTC.set(tm);

  showMenu(menuIndex, false);
}

void Watchy::showAccelerometer() {
//  display.setFullWindow();
//  display.fillScreen(GxEPD_BLACK);
//  display.setFont(&FreeMonoBold9pt7b);
//  display.setTextColor(GxEPD_WHITE);
//
//  Accel acc;
//
//  long previousMillis = 0;
//  long interval       = 200;
//
//  guiState = APP_STATE;
//
//  pinMode(BACK_BTN_PIN, INPUT);
//
//  while (1) {
//
//    unsigned long currentMillis = millis();
//
//    if (digitalRead(BACK_BTN_PIN) == 1) {
//      break;
//    }
//
//    if (currentMillis - previousMillis > interval) {
//      previousMillis = currentMillis;
//      // Get acceleration data
//      bool res          = sensor.getAccel(acc);
//      uint8_t direction = sensor.getDirection();
//      display.fillScreen(GxEPD_BLACK);
//      display.setCursor(0, 30);
//      if (res == false) {
//        display.println("getAccel FAIL");
//      } else {
//        display.print("  X:");
//        display.println(acc.x);
//        display.print("  Y:");
//        display.println(acc.y);
//        display.print("  Z:");
//        display.println(acc.z);
//
//        display.setCursor(30, 130);
//        switch (direction) {
//        case DIRECTION_DISP_DOWN:
//          display.println("FACE DOWN");
//          break;
//        case DIRECTION_DISP_UP:
//          display.println("FACE UP");
//          break;
//        case DIRECTION_BOTTOM_EDGE:
//          display.println("BOTTOM EDGE");
//          break;
//        case DIRECTION_TOP_EDGE:
//          display.println("TOP EDGE");
//          break;
//        case DIRECTION_RIGHT_EDGE:
//          display.println("RIGHT EDGE");
//          break;
//        case DIRECTION_LEFT_EDGE:
//          display.println("LEFT EDGE");
//          break;
//        default:
//          display.println("ERROR!!!");
//          break;
//        }
//      }
//      display.display(true); // full refresh
//    }
//  }
//
//  showMenu(menuIndex, false);
}

void Watchy::showWatchFace(bool partialRefresh) {
  display.setFullWindow();
  drawWatchFace();
  display.display(partialRefresh); // partial refresh
  guiState = WATCHFACE_STATE;
}

void Watchy::drawWatchFace() {
  display.setFont(&DSEG7_Classic_Bold_53);
  display.setCursor(5, 53 + 60);
  if (currentTime.Hour < 10) {
    display.print("0");
  }
  display.print(currentTime.Hour);
  display.print(":");
  if (currentTime.Minute < 10) {
    display.print("0");
  }
  display.println(currentTime.Minute);
}

weatherData Watchy::getWeatherData() {
  return getWeatherData(settings.cityID, settings.weatherUnit,
                        settings.weatherLang, settings.weatherURL,
                        settings.weatherAPIKey, settings.weatherUpdateInterval);
}

weatherData Watchy::getWeatherData(String cityID, String units, String lang,
                                   String url, String apiKey,
                                   uint8_t updateInterval) {
//  currentWeather.isMetric = units == String("metric");
//  if (weatherIntervalCounter < 0) { //-1 on first run, set to updateInterval
//    weatherIntervalCounter = updateInterval;
//  }
//  if (weatherIntervalCounter >=
//      updateInterval) { // only update if WEATHER_UPDATE_INTERVAL has elapsed
//                        // i.e. 30 minutes
//    if (connectWiFi()) {
//      HTTPClient http; // Use Weather API for live data if WiFi is connected
//      http.setConnectTimeout(3000); // 3 second max timeout
//      String weatherQueryURL = url + cityID + String("&units=") + units +
//                               String("&lang=") + lang + String("&appid=") +
//                               apiKey;
//      http.begin(weatherQueryURL.c_str());
//      int httpResponseCode = http.GET();
//      if (httpResponseCode == 200) {
//        String payload             = http.getString();
//        JSONVar responseObject     = JSON.parse(payload);
//        currentWeather.temperature = int(responseObject["main"]["temp"]);
//        currentWeather.weatherConditionCode =
//            int(responseObject["weather"][0]["id"]);
//        currentWeather.weatherDescription =
//	  JSONVar::stringify(responseObject["weather"][0]["main"]);
//	    currentWeather.external = true;
//        // sync NTP during weather API call and use timezone of city
//        gmtOffset = int(responseObject["timezone"]);
//        syncNTP(gmtOffset);
//      } else {
//        // http error, use internal temperature sensor
//        currentWeather.temperature          = currentWeather.isMetric ? (uint8_t)sensor.readTemperature() : (uint8_t)sensor.readTemperatureF();
//        currentWeather.weatherConditionCode = -1;
//        currentWeather.external             = false;        
//      }
//      http.end();
//      // turn off radios
//      WiFi.mode(WIFI_OFF);
//      btStop();
//    } else { // No WiFi, use internal temperature sensor
//      currentWeather.temperature          = currentWeather.isMetric ? (uint8_t)sensor.readTemperature() : (uint8_t)sensor.readTemperatureF();
//      currentWeather.weatherConditionCode = -1;
//      currentWeather.external             = false;
//    }
//    weatherIntervalCounter = 0;
//  } else {
//    weatherIntervalCounter++;
//  }
//  return currentWeather;
}

float Watchy::getBatteryVoltage() {
  if (RTC.rtcType == DS3231) {
    return analogReadMilliVolts(BATT_ADC_PIN) / 1000.0f *
           2.0f; // Battery voltage goes through a 1/2 divider.
  } else {
    return analogReadMilliVolts(BATT_ADC_PIN) / 1000.0f * 2.0f;
  }
}

void bma4xx_hal_delay_usec(uint32_t period_us, void *intf_ptr)
{
  delayMicroseconds(period_us);
}
/*! This API is used to perform I2C read operation with sensor */
int8_t bma4xx_hal_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //uint8_t dev_id = 0x68;
  uint8_t* dev_id = (uint8_t *)intf_ptr;

  rslt = BMA456_read_i2c(*dev_id, reg_addr, reg_data, length);

  return rslt;
}

/*! This API is used to perform I2C write operations with sensor */
int8_t bmi4xx_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //    uint8_t dev_id = 0x68;

  uint8_t* dev_id = (uint8_t *)intf_ptr;
  rslt = BMA456_write_i2c(*dev_id, reg_addr, (uint8_t *)reg_data, length);

  return rslt;
}

int8_t BMA456_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /* dev_addr: I2C device address.
    reg_addr: Starting address for writing the data.
    reg_data: Data to be written.
    count: Number of bytes to write */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();

  if (returned)
  {
    /*
      case 1:Data too long to fit in transmit buffer
          break;
      case 2:received NACK on transmit of address.
          break;
      case 3:received NACK on transmit of data."
          break;
      case 4:Unspecified error.
          break;
      default:Unexpected Wire.endTransmission() return code:
    */
    return returned;
  }

  // Requests the required number of bytes from the sensor
  Wire.requestFrom((int)dev_addr, (int)count);

  uint16_t i;
  // Reads the requested number of bytes into the provided array
  for (i = 0; (i < count) && Wire.available(); i++)
  {
    reg_data[i] = Wire.read(); // This is for the modern Wire library
  }

  // This must return 0 on success, any other value will be interpreted as a communication failure.
  return 0;
}

int8_t BMA456_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /*  dev_addr: I2C device address.
    reg_addr: Starting address for reading the data.
    reg_data: Buffer to take up the read data.
    count: Number of bytes to read. */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);

  uint16_t i;
  // Writes the requested number of bytes from the provided array
  for (i = 0; i < count; i++)
  {
    Wire.write(reg_data[i]); // This is for the modern Wire library
  }
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();
  /*
      case 1:Data too long to fit in transmit buffer
      case 2:received NACK on transmit of address.
      case 3:received NACK on transmit of data.
      case 4:Unspecified error.
      default:Unexpected Wire.endTransmission() return code:
  */
  // This must return 0 on sucess, any other value will be interpretted as a communication failure.
  return returned;
}

int8_t bma4_interface_selection(struct bma4_dev *bma)
{
  int8_t rslt = BMA4_OK;

  if (bma != NULL)
  {
    /* Select the interface for execution
       For I2C : BMA4_I2C_INTF
       For SPI : BMA4_SPI_INTF
    */
    bma->intf = BMA4_I2C_INTF;

    /* Bus configuration : I2C */
    if (bma->intf == BMA4_I2C_INTF)
    {
      Serial.println("I2C Interface \n");

      /* To initialize the user I2C function */
      Wire.begin();
      dev_addr = BMA4_I2C_ADDR_PRIMARY;
      bma->bus_read = bma4xx_hal_i2c_bus_read;
      bma->bus_write = bmi4xx_hal_i2c_bus_write;
    }

    /* Bus configuration : SPI */
    else if (bma->intf == BMA4_SPI_INTF)
    {
      Serial.println("SPI Interface \n");

      /* To initialize the user SPI function */
      SPI.begin();
      dev_addr = 0;
      //bma->bus_read = user_spi_read;
      //bma->bus_write = user_spi_write;
    }

    /* Assign device address to interface pointer */
    bma->intf_ptr = &dev_addr;

    /* Configure delay in microseconds */
    bma->delay_us = bma4xx_hal_delay_usec;

    /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
    bma->read_write_len = 8;
  }
  else
  {
    rslt = BMA4_E_NULL_PTR;
  }

  return rslt;

}


void Watchy::_bmaConfig() {
//
//  if (sensor.begin(_readRegister, _writeRegister, delay) == false) {
//    // fail to init BMA
//    return;
//  }
//
//  // Accel parameter structure
//  Acfg cfg;
//  /*!
//      Output data rate in Hz, Optional parameters:
//          - BMA4_OUTPUT_DATA_RATE_0_78HZ
//          - BMA4_OUTPUT_DATA_RATE_1_56HZ
//          - BMA4_OUTPUT_DATA_RATE_3_12HZ
//          - BMA4_OUTPUT_DATA_RATE_6_25HZ
//          - BMA4_OUTPUT_DATA_RATE_12_5HZ
//          - BMA4_OUTPUT_DATA_RATE_25HZ
//          - BMA4_OUTPUT_DATA_RATE_50HZ
//          - BMA4_OUTPUT_DATA_RATE_100HZ
//          - BMA4_OUTPUT_DATA_RATE_200HZ
//          - BMA4_OUTPUT_DATA_RATE_400HZ
//          - BMA4_OUTPUT_DATA_RATE_800HZ
//          - BMA4_OUTPUT_DATA_RATE_1600HZ
//  */
//  cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
//  /*!
//      G-range, Optional parameters:
//          - BMA4_ACCEL_RANGE_2G
//          - BMA4_ACCEL_RANGE_4G
//          - BMA4_ACCEL_RANGE_8G
//          - BMA4_ACCEL_RANGE_16G
//  */
//  cfg.range = BMA4_ACCEL_RANGE_2G;
//  /*!
//      Bandwidth parameter, determines filter configuration, Optional parameters:
//          - BMA4_ACCEL_OSR4_AVG1
//          - BMA4_ACCEL_OSR2_AVG2
//          - BMA4_ACCEL_NORMAL_AVG4
//          - BMA4_ACCEL_CIC_AVG8
//          - BMA4_ACCEL_RES_AVG16
//          - BMA4_ACCEL_RES_AVG32
//          - BMA4_ACCEL_RES_AVG64
//          - BMA4_ACCEL_RES_AVG128
//  */
//  cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
//
//  /*! Filter performance mode , Optional parameters:
//      - BMA4_CIC_AVG_MODE
//      - BMA4_CONTINUOUS_MODE
//  */
//  cfg.perf_mode = BMA4_CONTINUOUS_MODE;
//
//  // Configure the BMA423 accelerometer
//  sensor.setAccelConfig(cfg);
//
//  // Enable BMA423 accelerometer
//  // Warning : Need to use feature, you must first enable the accelerometer
//  // Warning : Need to use feature, you must first enable the accelerometer
//  sensor.enableAccel();
//
//  struct bma4_int_pin_config config;
//  config.edge_ctrl = BMA4_LEVEL_TRIGGER;
//  config.lvl       = BMA4_ACTIVE_HIGH;
//  config.od        = BMA4_PUSH_PULL;
//  config.output_en = BMA4_OUTPUT_ENABLE;
//  config.input_en  = BMA4_INPUT_DISABLE;
//  // The correct trigger interrupt needs to be configured as needed
//  sensor.setINTPinConfig(config, BMA4_INTR1_MAP);
//
//  struct bma423_axes_remap remap_data;
//  remap_data.x_axis      = 1;
//  remap_data.x_axis_sign = 0xFF;
//  remap_data.y_axis      = 0;
//  remap_data.y_axis_sign = 0xFF;
//  remap_data.z_axis      = 2;
//  remap_data.z_axis_sign = 0xFF;
//  // Need to raise the wrist function, need to set the correct axis
//  sensor.setRemapAxes(&remap_data);
//
//  // Enable BMA423 isStepCounter feature
//  sensor.enableFeature(BMA423_STEP_CNTR, true);
//  // Enable BMA423 isTilt feature
//  sensor.enableFeature(BMA423_TILT, true);
//  // Enable BMA423 isDoubleClick feature
//  sensor.enableFeature(BMA423_WAKEUP, true);
//
//  // Reset steps
//  sensor.resetStepCounter();
//
//  // Turn on feature interrupt
//  sensor.enableStepCountInterrupt();
//  sensor.enableTiltInterrupt();
//  // It corresponds to isDoubleClick interrupt
//  sensor.enableWakeupInterrupt();
}

void Watchy::setupWifi() {
  display.epd2.setBusyCallback(0); // temporarily disable lightsleep on busy
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  wifiManager.setTimeout(WIFI_AP_TIMEOUT);
  wifiManager.setAPCallback(_configModeCallback);
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  if (!wifiManager.autoConnect(WIFI_AP_SSID)) { // WiFi setup failed
    display.println("Setup failed &");
    display.println("timed out!");
  } else {
    display.println("Connected to");
    display.println(WiFi.SSID());
		display.println("Local IP:");
		display.println(WiFi.localIP());
    weatherIntervalCounter = -1; // Reset to force weather to be read again
  }
  display.display(false); // full refresh
  // turn off radios
  WiFi.mode(WIFI_OFF);
  btStop();
  display.epd2.setBusyCallback(displayBusyCallback); // enable lightsleep on
                                                     // busy
  guiState = APP_STATE;
}

void Watchy::_configModeCallback(WiFiManager *myWiFiManager) {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Connect to");
  display.print("SSID: ");
  display.println(WIFI_AP_SSID);
  display.print("IP: ");
  display.println(WiFi.softAPIP());
	display.println("MAC address:");
	display.println(WiFi.softAPmacAddress().c_str());
  display.display(false); // full refresh
}

bool Watchy::connectWiFi() {
  if (WL_CONNECT_FAILED ==
      WiFi.begin()) { // WiFi not setup, you can also use hard coded credentials
                      // with WiFi.begin(SSID,PASS);
    WIFI_CONFIGURED = false;
  } else {
    if (WL_CONNECTED ==
        WiFi.waitForConnectResult()) { // attempt to connect for 10s
      WIFI_CONFIGURED = true;
    } else { // connection failed, time out
      WIFI_CONFIGURED = false;
      // turn off radios
      WiFi.mode(WIFI_OFF);
      btStop();
    }
  }
  return WIFI_CONFIGURED;
}

void Watchy::showUpdateFW() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Please visit");
  display.println("watchy.sqfmi.com");
  display.println("with a Bluetooth");
  display.println("enabled device");
  display.println(" ");
  display.println("Press menu button");
  display.println("again when ready");
  display.println(" ");
  display.println("Keep USB powered");
  display.display(false); // full refresh

  guiState = FW_UPDATE_STATE;
}

void Watchy::updateFWBegin() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Bluetooth Started");
  display.println(" ");
  display.println("Watchy BLE OTA");
  display.println(" ");
  display.println("Waiting for");
  display.println("connection...");
  display.display(false); // full refresh

  BLE BT;
  BT.begin("Watchy BLE OTA");
  int prevStatus = -1;
  int currentStatus;

  while (1) {
    currentStatus = BT.updateStatus();
    if (prevStatus != currentStatus || prevStatus == 1) {
      if (currentStatus == 0) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("BLE Connected!");
        display.println(" ");
        display.println("Waiting for");
        display.println("upload...");
        display.display(false); // full refresh
      }
      if (currentStatus == 1) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("Downloading");
        display.println("firmware:");
        display.println(" ");
        display.print(BT.howManyBytes());
        display.println(" bytes");
        display.display(true); // partial refresh
      }
      if (currentStatus == 2) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("Download");
        display.println("completed!");
        display.println(" ");
        display.println("Rebooting...");
        display.display(false); // full refresh

        delay(2000);
        esp_restart();
      }
      if (currentStatus == 4) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("BLE Disconnected!");
        display.println(" ");
        display.println("exiting...");
        display.display(false); // full refresh
        delay(1000);
        break;
      }
      prevStatus = currentStatus;
    }
    delay(100);
  }

  // turn off radios
  WiFi.mode(WIFI_OFF);
  btStop();
  showMenu(menuIndex, false);
}

void Watchy::showSyncNTP() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Syncing NTP... ");
  display.print("GMT offset: ");
  display.println(gmtOffset);
  display.display(false); // full refresh
  if (connectWiFi()) {
    if (syncNTP()) {
      display.println("NTP Sync Success\n");
      display.println("Current Time Is:");

      RTC.read(currentTime);

      display.print(tmYearToCalendar(currentTime.Year));
      display.print("/");
      display.print(currentTime.Month);
      display.print("/");
      display.print(currentTime.Day);
      display.print(" - ");

      if (currentTime.Hour < 10) {
        display.print("0");
      }
      display.print(currentTime.Hour);
      display.print(":");
      if (currentTime.Minute < 10) {
        display.print("0");
      }
      display.println(currentTime.Minute);
    } else {
      display.println("NTP Sync Failed");
    }
  } else {
    display.println("WiFi Not Configured");
  }
  display.display(true); // full refresh
  delay(3000);
  showMenu(menuIndex, false);
}

bool Watchy::syncNTP() { // NTP sync - call after connecting to WiFi and
                         // remember to turn it back off
  return syncNTP(gmtOffset,
                 settings.ntpServer.c_str());
}

bool Watchy::syncNTP(long gmt) {
  return syncNTP(gmt, settings.ntpServer.c_str());
}

bool Watchy::syncNTP(long gmt, String ntpServer) {
  // NTP sync - call after connecting to
  // WiFi and remember to turn it back off
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, ntpServer.c_str(), gmt);
  timeClient.begin();
  if (!timeClient.forceUpdate()) {
    return false; // NTP sync failed
  }
  tmElements_t tm;
  breakTime((time_t)timeClient.getEpochTime(), tm);
  RTC.set(tm);
  return true;
}
