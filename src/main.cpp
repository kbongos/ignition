// Ignitor.cpp - ignition for small engine with ADV and F timing marks
//  Initially for my 1981 CM400T now with pulse signals in to stock AC-CDI.
// Todo: Make so it can log stats to flash, on demand or stop or something.
//   Rev limit useful?  Is it a needed thing?
//   Add support for 12V TCI, points.  Add Honda CB350/CB450 180 dual TCI ignition support.

//#define TARGET_X86
//#define TARGET_D1_MINI
#ifdef ESP8266
//#define TARGET_X86 // to force visual code to see X86 code, wish I had better way
#define TARGET_D1_MINI
#else
#define TARGET_X86
#endif

#ifdef TARGET_X86
#include <iostream>
#include <vector>
#include <string>
#include <stdint.h>
using namespace std;

uint32_t sim_us_tm = 0;
#define SIM_IO_F 1
#define SIM_IO_A 2
#define SIM_IO_SW1 4
#define SIM_IO_SW2 8
#define SIM_IO_i1 0x10
#define SIM_IO_i2 0x20
#define SIM_IO_L0 0x40
#define SIM_IO_L1 0x80
#define SIM_IO_L2 0x100
uint16_t sim_io = 0;
uint16_t sim_io_last = 0; // change detect
#endif


#ifdef TARGET_D1_MINI
#define Serial_begin Serial.begin
#define Serial_println Serial.println
#ifdef DO_WIFI_SERVER
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#else
#include <Arduino.h>
#include <ESP8266WiFi.h>
#endif
#endif

#ifdef TARGET_X86
//#include <Arduino.h>
//#include <ESP8266WiFi.h>
using namespace std;
#include <stdint.h>
#include <stdio.h>
#endif


#define DO_SERIAL
//#define DO_FS
#ifdef DO_FS
#include "FS.h"
#endif
//#define DO_RAMLOG

#ifdef DO_RAMLOG
#define RAMLOG_SZ 10000
uint16_t ramlog_i = 0;
char ramlog[RAMLOG_SZ];
#endif

const int io_led0     = 2; // IO2(D4), low LED on (D1-Mini built in)
const int io_in_sw2  = 5; // IO5(D1), , low on
const int io_in_sw1  = 4; // IO4(D2), , low on
// IO8(D8) - configured as input by mistake caused no-run
// IO15 seems pulled to ground, something > 10K < 100K.  pull-up on others > 10K < 100K
//const int io_in_ign  =  5; // IO5(D1), low active, high active
const int io_in_f    = 13; // IO13(D7), low active, fire(idle) mark in
const int io_in_a = 12; // IO12(D6), , max advance mark, in
const int io_out_ign2 = 15; // IO15(D8), low on, 2nd fire ignitor 2

//#define VER2_BRD
#define VER1_BRD
#ifdef VER1_BRD
const int io_out_ign1  = 14; // IO14(D5), low on
const int io_led1  = 16; // IO16(D0), drive high
#endif
#ifdef VER2_BRD
// Rearranged a few outputs for ease of layout on SMD board only
const int io_led1  = 14; // IO14(D5), drive high
const int io_out_ign1  = 16; // IO16(D0), drive high
#endif

// These are usec time constants are length of ignitor on time
#define TCI_DWELL_TIME 2000 // For TCI mode(use_tci=1)
#define CDI_TRIGGER_TIME 1000 // For CDI mode(use_tci=0)

//#define TM_MASK 0xffffffff
#define TM_MASK 0xffffff
uint32_t tm_last = 0; // a place to store last usec time reading from system

// Slight problem using 16-bit, at 1500rpm us rev count is 40000us.  So getting below
// that some it will overflow somewhere around 1200rpm.  We do limit count up now, so might
// be ok as we just fire at F then.  Not like we have to calculate adv then, but hurts
// statistics at a minimum for starting speeds.

uint16_t cnt1_us_timebase = 0; // us count for general timing, counts up
uint16_t cnt2_us_timebase = 0; // us count for general timing, counts up

uint16_t cnt_f_us_revbase = 0; // us count up since last F
uint16_t cnt_f_us_revlast = 0; // last us count of F to F

uint16_t cnt_adv_us_revbase = 0; // us count up since last ADV
uint16_t cnt_adv_us_revlast = 0; // last us count of ADV to ADV

uint16_t cnt2_us_af_base = 0; // us count up since last A
uint16_t cnt2_us_af_last = 0; // last us count of F to F

uint16_t cnt1_100us_timebase = 0; // general timing, bump up, reset cnt1_us_timebase
uint16_t cnt1_ms_timebase = 0; // general timing, bump up, reset cnt1_100us_timebase
uint16_t cnt1_s_timebase = 0; // general timing, bump up, reset cnt1_ms_timebase
uint16_t cnt2_10ms_timebase = 0;

uint16_t cnt_revs_stats = 0; // count of revs, for stats, reset on stats
uint16_t rev_us_min_stats = 0; // minimum rev measure, for stats, reset on stats
uint16_t af_us_min_stats = 0; // minimum A to F measure, for stats, reset on stats
uint16_t rev_us_max_stats = 0; // minimum rev measure, for stats, reset on stats
uint16_t af_us_max_stats = 0; // minimum A to F measure, for stats, reset on stats
uint32_t af_us_total_stats = 0; // total for average, A-F time, for stats, reset on stats
uint32_t rev_us_total_stats = 0; // total for average, F-F(rev), for stats, reset on stats
// Attempt to track unexpected signal timing:
uint16_t rev_us_error_cnt = 0; // Error count, F to F(rev), rev expected window
uint16_t af_us_error_cnt = 0; // Error count, A to F, rev expected window

uint16_t tim_us_last_rev = 0; // time of last rev from F to F
uint16_t tim_us_last_af = 0; // time between A to F

uint8_t in_sw1; // = digitalRead(io_in_sw1);
uint8_t in_sw2; // = digitalRead(io_in_sw2);
uint8_t last_in_sw1 = 0; // Detect change
uint8_t last_in_sw2 = 0; // Detect change
uint8_t opt_gen = 0; // true if sw2 low to turn on test signal generator
#ifdef DO_FS
uint8_t use_fs = 0; // true if using file system for log
#endif
uint8_t cfg_gen_lo = 0; // true if signal gen pulse low, otherwise high
uint8_t perform_1s_stats_request = 0; // 1H=1 second stats/work, 2H=4second stats/work

uint16_t tci_ignitor_dwell_in_us = 0; // TCI use, time in us since start ignitor/dwell
uint16_t ignitor_in_us = 0; // time in us to delay and then turn on ignitor
uint16_t last_ignitor_in_us = 0; // last set to show in stats

#define BOOL_t uint8_t
const BOOL_t use_f_edges = 0; // option 1=use only single input sensor, leading=A, trailing=F
const BOOL_t use_f_neg = 0; // 0= pulse is positive reading
const BOOL_t use_adv_neg = 0; // 0= pulse is positive reading
const BOOL_t use_ign_pos = 1; // 1= pulse is positive on
const BOOL_t use_led1_pos = 1; // 1= on when positive
const BOOL_t use_tci = 0; // 1= on, use TCI(12v) long dwell[EXPERIMENTAL!], otherwise assume CDI fires immediately

BOOL_t last_in_f = use_f_neg; // digital state of last F reading(1/0).  Detect change
BOOL_t last_in_a = use_adv_neg; // digital state of last A reading(1/0).  Detect change

uint8_t ev = 0; // event, bits set for event
#define EV_A_MARK 1
#define EV_F_MARK 2
#define SW1_ON_0 0
#define SW2_ON_0 0
uint16_t fire_ign = 0; // boolean and us timer to indicate when Ignitor on
uint32_t tim_us_now = 0; // base time measurements, current reading now
uint32_t tim_us_diff = 0; // base time measurements, difference from last reading

uint16_t percent_af_adv; // percent A-F time to after A to start ignitor based on advance
uint16_t gen_100us_base = 400; // Generator rpm time in .1ms  400=40ms(1.5krpm). 200=3krpm, 100=6krpm.
uint8_t gen_auto_ramp = 1; // Generator auto ramp between idle and 6k rpm for example.
uint16_t in_f_cnt_filter = 30000; // filter accumulators on F mark input, us based
uint16_t in_a_cnt_filter = 30000; // filter accumulators on ADV mark input, us based

uint16_t time_print_us_stats = 0; // time measured for time to print stats
//#define DO_JITTER_STATS
#ifdef DO_JITTER_STATS
uint16_t max_tim_us_diff_stats = 0;
uint16_t min_tim_us_diff_stats = 0;
uint32_t total_tim_us_diff_stats = 0;
uint32_t total_tim_us_diff_count = 0;
#endif
uint8_t did_pr_stats_ignore_is_diff_stats = 0; // set to true after print stats done to ignore min,max calc of us diff
#define F_USED_CNT_NUM_40 40
uint8_t f_use_cnt = F_USED_CNT_NUM_40; // count down some amount of F fires before allowing advance timing.
bool too_slow_rpm_use_f_fire = false; // flag to fire on F when advance not at slow RPM
uint16_t perc_x10_af_angle = 300; // Angle x10 in percent rev, from ADV to F measure at start
char logln[160];

#ifdef TARGET_X86
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT 3
#define OUTPUT_OPEN_DRAIN 4

void pinMode(int io, int md)
{
}

void digitalWrite(int io, int md)
{
  if (io == io_led0)
    sim_io  = md ? (sim_io | SIM_IO_L0) : (sim_io & ~SIM_IO_L0);
  else if (io == io_led1)
    sim_io  = md ? (sim_io | SIM_IO_L1) : (sim_io & ~SIM_IO_L1);
  else if (io == io_out_ign1)
    sim_io  = md ? (sim_io | SIM_IO_i1) : (sim_io & ~SIM_IO_i1);
  else if (io == io_out_ign2)
    sim_io  = md ? (sim_io | SIM_IO_i2) : (sim_io & ~SIM_IO_i2);
}

uint32_t digitalRead(int io)
{
  if (io == io_in_f)
    return sim_io & SIM_IO_F;
  else if (io == io_in_a)
    return sim_io & SIM_IO_A;
  else if (io == io_in_sw1)
    return sim_io & SIM_IO_SW1;
  else if (io == io_in_sw2)
    return sim_io & SIM_IO_SW2;

  return 0;
}

void Serial_begin(int baud)
{
}
void  Serial_println(const char *msg)
{
  printf("%s\n", msg);
}
uint64_t system_get_time(void)
{
  return sim_us_tm;
}

void system_soft_wdt_feed(void)
{
}
void delayMicroseconds(uint32_t usec)
{
  sim_us_tm += usec;
}
#endif

#ifdef DO_WIFI_SERVER
ESP8266WebServer server(80);
const char* ssid = "YourWifi";
const char* password = "";
#endif
#ifdef DO_WIFI_SERVER

void handleRoot() {
  digitalWrite(io_led1, 1);
  server.send(200, "text/plain", "hello from esp8266!");
  digitalWrite(io_led1, 0);
}

void handleNotFound(){
  digitalWrite(io_led1, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(io_led1, 0);
}
#endif

void setup(void)
{
  pinMode(io_led0, OUTPUT);
  pinMode(io_led1, OUTPUT);
  //pinMode(io_led3, OUTPUT); OUTPUT_OPEN_DRAIN
  pinMode(io_out_ign1, OUTPUT);
  pinMode(io_out_ign2, OUTPUT_OPEN_DRAIN);
  pinMode(io_in_sw1, INPUT_PULLUP);
  pinMode(io_in_sw2, INPUT_PULLUP);
  pinMode(io_in_f, INPUT);
  pinMode(io_in_a, INPUT);

  digitalWrite(io_led0, 0);
  Serial_begin(115200);

#ifdef DO_WIFI_SERVER
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Ignitor Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
#endif
}

#ifdef DO_FS
//-----------------------------------
void write_log(const char *msg, bool reset)
{
    File f = SPIFFS.open("/log.txt", reset ? "w" : "a");
    if (!f)
      Serial.println("wr log open failed");
    else
    {
      f.println(msg);
      f.close();
    }
}

//-----------------------------------
void dump_log()
{
    File f = SPIFFS.open("/log.txt", "r");
    if (!f)
      Serial.println("rd log open failed");
    else
    {
      system_soft_wdt_feed(); // Kick watchdog or it will timeout after a few seconds and restart
      while(f.available()) {
        Serial.println(f.readStringUntil('\n'));
        //Serial.println(line);
      }      
      f.close();
    }
}
#endif

#ifdef DO_RAMLOG
//-----------------------------------
void dump_ramlog()
{
  system_soft_wdt_feed(); // Kick watchdog or it will timeout after a few seconds and restart
  Serial.write(ramlog, ramlog_i);
  ramlog_i = 0;
}
#endif

//-----------------------------------
void log(const char *msg)
{
#ifdef DO_RAMLOG
if (ramlog_i < (RAMLOG_SZ-120))
{
   uint16_t msg_sz = strlen(msg);
   memcpy(&ramlog[ramlog_i], msg, msg_sz);
   ramlog_i += msg_sz;
   ramlog[ramlog_i++] = 0xd;
   ramlog[ramlog_i++] = 0xa;
   ramlog[ramlog_i] = 0;
}
#else
#ifdef DO_SERIAL
  Serial_println(msg);
#endif
#endif

#ifdef DO_FS
  if (use_fs)
    write_log(msg, 0);
#endif
}

//-----------------------------------
void pr_u16(const char *msg, uint16_t ui)
{
  sprintf(logln, "%s:%u", msg, ui);
  log(logln);
}
//-----------------------------------
void pr_h16(const char *msg, uint16_t ui)
{
  sprintf(logln, "%s:0x%x", msg, ui);
  log(logln);
}
//-----------------------------------
void pr_d16(const char *msg, int16_t i)
{
  sprintf(logln, "%s:%d", msg, i);
  log(logln);
}

//-----------------------------------
void pr_u32(const char *msg, uint32_t ui)
{
  sprintf(logln, "%s:%u", msg, ui);
  log(logln);
}
//-----------------------------------
void pr_h32(const char *msg, uint32_t ui)
{
  sprintf(logln, "%s:0x%x", msg, ui);
  log(logln);
}
//-----------------------------------
void pr_d32(const char *msg, int32_t i)
{
  sprintf(logln, "%s:%d", msg, i);
  log(logln);
}

//-----------------------------------
void ignitor_engage(void)
{
  digitalWrite(io_out_ign1, use_ign_pos);
  digitalWrite(io_out_ign2, use_ign_pos);
}

//-----------------------------------
void led0_engage(void)
{
  digitalWrite(io_led0, 0);
}

//-----------------------------------
void led2_engage(void)
{
  digitalWrite(io_led1, use_led1_pos ? 1 : 0);
}

//-----------------------------------
//void led3_engage(void)
//{
//  digitalWrite(io_led3, 1);
//}

//-----------------------------------
void ignitor_disengage(void)
{
  digitalWrite(io_out_ign1, use_ign_pos ? 0 : 1);
  digitalWrite(io_out_ign2, use_ign_pos ? 0 : 1);
}

//-----------------------------------
void led0_disengage(void)
{
  digitalWrite(io_led0, 1);
}

//-----------------------------------
void led2_disengage(void)
{
  digitalWrite(io_led1, use_led1_pos ? 0 : 1);
}

//-----------------------------------
//void led3_disengage(void)
//{
//  digitalWrite(io_led3, 0);
//}

//-----------------------------------
void ignitor_disengage_work(void)
{
    if (fire_ign > tim_us_diff)
      fire_ign -= tim_us_diff;
    else
      fire_ign = 0;
    if (fire_ign == 0)
    {
      ignitor_disengage();
    }
}

//-----------------------------------
void tci_dwell_disengage_work(void)
{
    tci_ignitor_dwell_in_us += tim_us_diff;
    if (tci_ignitor_dwell_in_us > (TCI_DWELL_TIME*8)) // unexpected, fire position not reached in overlong dwell period
    {
      tci_ignitor_dwell_in_us = 0; // turn off dwell work
      ignitor_disengage();
    }
}

//-----------------------------------
// time down @ignitor_in_us and engage ignitor when time up
void ign_engage_work(void)
{
    if (ignitor_in_us > tim_us_diff)
      ignitor_in_us -= tim_us_diff;
    else
      ignitor_in_us = 0;
    if (ignitor_in_us == 0) // time to fire
    {
      if (use_tci)
      {
        // This is hard to understand, trying to engage by dis-engaging with TCI
        if (tci_ignitor_dwell_in_us > 0)
        {
          if (tci_ignitor_dwell_in_us >= TCI_DWELL_TIME)
            ignitor_disengage();
          else
          {
            fire_ign = TCI_DWELL_TIME - tci_ignitor_dwell_in_us;
          }
        }
      }
      else
      {
         fire_ign = CDI_TRIGGER_TIME;
         ignitor_engage();
      }
    }
}

//-----------------------------------
// Switch 1 is used to allow advance timing operation.  When off, it always fires at (F),
// when on we allow to fire at an advance(A) point between F and A.
bool is_sw1_adv_on()
{
  return (in_sw1 == SW1_ON_0); // use adv
}

//-----------------------------------
// Switch 2 is used to active test signal generator mode when on(active).
bool is_sw2_on()
{
  return (in_sw2 == SW2_ON_0); // on startup, set generator mode.  At operation undefined.
}

//-----------------------------------
// Return true if advance is not used at the moment and only firing at F is appropriate.
bool is_only_f_used()
{
  // if SW1 is OFF(Do not use advance), or if are still counting down initial f-only revs
  if (!is_sw1_adv_on() || f_use_cnt != 0)
     return true; // return true, only fire at F, no advance please
  return false; // feel free to fire at advanced timing before F mark(and after ADV mark)
}

//-----------------------------------
// Perform every 1 seconds work
void work_1s()
{
  // TODO: this needs to be synced appropriately, this is primarily for stats.
  // Perhaps use a more dedicated counter would be better!
  // We need some better sense when starting at less than firing RPM.
  if (cnt_revs_stats == 0) // no revs counted for second, stopped
  {
     // reset so F has to be used before advance considered
     f_use_cnt = F_USED_CNT_NUM_40;
  }
}

//-----------------------------------
// Perform every 4 seconds work
void work_4s(void)
{
  // printing this to serial port takes a certain amount of time.
  // logging to flash can take considerably more, and it varys based on HW
  // So we try to arrange this at a time when it does not harm our timing
  sprintf(logln, "r:%d,%d,%d(%d) af:%d,%d,%d(%d) rc:%d re:%d ae:%d lpr:%d adv:%d igus:%d",
    rev_us_min_stats, cnt_revs_stats > 0 ? rev_us_total_stats/cnt_revs_stats : 0, rev_us_max_stats,
    rev_us_max_stats - rev_us_min_stats,
    af_us_min_stats, cnt_revs_stats > 0 ? af_us_total_stats/cnt_revs_stats : 0, af_us_max_stats,
    af_us_max_stats - af_us_min_stats,
    cnt_revs_stats, rev_us_error_cnt, af_us_error_cnt, time_print_us_stats, percent_af_adv, last_ignitor_in_us);
  log(logln);
#ifdef DO_JITTER_STATS
  sprintf(logln, " t16:%d,%d,%d(%d,%d)",
    min_tim_us_diff_stats, total_tim_us_diff_count > 0 ? total_tim_us_diff_stats/total_tim_us_diff_count : 0, max_tim_us_diff_stats,
    max_tim_us_diff_stats - min_tim_us_diff_stats,
    total_tim_us_diff_count);
  log(logln);
  total_tim_us_diff_stats = 0;
  total_tim_us_diff_count = 0;
  max_tim_us_diff_stats = 0;
  min_tim_us_diff_stats = 65000;
#endif
  // reset our stat counters used for ave,min,max
  rev_us_error_cnt = 0;
  af_us_error_cnt = 0;

  cnt_revs_stats = 0;
  rev_us_total_stats = 0;
  af_us_total_stats = 0;
  rev_us_min_stats = 59999;
  af_us_min_stats = 59999;
  rev_us_max_stats = 0;
  af_us_max_stats = 0;
  did_pr_stats_ignore_is_diff_stats = 1;
  // this is to get a measure of how long this routine takes and show
  time_print_us_stats = (system_get_time() & TM_MASK) - tim_us_now; // ignore roll over for now
}

//-----------------------------------
// Event detect, read our IO, allow some filtering and set flags to show when IO changes.
void ev_detect(void)
{
  // This is attempt at debounce filtering
  static const uint16_t filter_constant_on = 25; //us
  static const uint16_t filter_constant_off = 25; // us

  if (use_f_edges) // use only leading edge(A), trailing edge(F) for timing.
  // Assume this is on F input, not ADV input.
  {
#if 0
    // not using at the moment, to come back and code for leading/trailing A,F work.
    uint16_t in_f = digitalRead(io_in_f);
    if (in_f != last_in_f)
    {
      if (in_f == 0) // went to 0(leading edge ADV)
      {
         ev |= EV_A_MARK;
         last_in_f = in_f;
      } // else went to 1(trailing edge F)
      else if (in_f == 1)
      {
        ev |= EV_F_MARK;
        last_in_f = in_f;
      }
      //pr_u16("F EV", in_f);
    }
    // Not sure what i'm doing with A input here
    uint16_t in_a = digitalRead(io_in_a);

    if (in_a != last_in_a)
    {
      if (in_a == 0) // went to 0(leading edge ADV)
      {
         ev |= EV_A_MARK;
         last_in_a = in_a;
      } // else went to 1(trailing edge ADV)
      else if (in_a == 1)
      {
        ev |= EV_F_MARK;
        last_in_a = in_a;
      }
      //pr_u16("A EV", in_a);
    }
#endif    
  }
  else // use separate input signals for (A) and (F) timing marks
  {
    BOOL_t in_f = digitalRead(io_in_f);
    // pulse active on low(short duration), off is high, long duration
    if (in_f != last_in_f)
    {
      if (in_f_cnt_filter > ((last_in_f == use_f_neg) ? filter_constant_on : filter_constant_off))
      {
        if (last_in_f == use_f_neg)
          ev |= EV_F_MARK;
        last_in_f = in_f;
      }
      else
        in_f_cnt_filter += tim_us_diff;
    }
    else if (in_f_cnt_filter > tim_us_diff)
      in_f_cnt_filter -= tim_us_diff;

    BOOL_t in_a = digitalRead(io_in_a);
    if (in_a != last_in_a)
    {
      if (in_a_cnt_filter > ((last_in_a == use_adv_neg) ? filter_constant_on : filter_constant_off))
      {
        if (last_in_a == use_adv_neg)
          ev |= EV_A_MARK;
        last_in_a = in_a;
        //pr_h16("ina", in_a);
      }
      else
        in_a_cnt_filter += tim_us_diff;
    }
    else if (in_a_cnt_filter > tim_us_diff)
      in_a_cnt_filter -= tim_us_diff;
  }
}

//-----------------------------------
void ign_work(void)
{
  if (ignitor_in_us > 0)
    ign_engage_work();

  if (ev != 0)
  {
    if (ev & EV_A_MARK)
    {
      if (cnt2_us_af_base > 4000) // ignore filter, us, after last ADV, crude and questionable
      {
        cnt2_us_af_base = 0; // start counting us up to F
        if (!opt_gen)
          led2_engage(); // led2 will show mirror of A(on) and F(off).  A sanity check.
        if (!is_only_f_used()) // use adv
        {
          tim_us_last_rev = cnt_adv_us_revbase;
          cnt_adv_us_revbase = 0;
          cnt_revs_stats += 1;

          // 100ms=600 50ms=1.2k 30ms=2k 20ms=3k 15ms=4k 12ms=5k 10ms=6k
          // tim_us_last_af
          const uint16_t min_rpm_max_us = 30000; // 2k rpm
          const uint16_t max_rpm_min_us = 15000; // 4k rpm
          const uint16_t range_us = min_rpm_max_us - max_rpm_min_us; // example: 20k
          const uint16_t af_us_div100 = range_us / 100; // example: 200
          if (tim_us_last_rev >= min_rpm_max_us) // too slow RPM for any advance
          {
            //ignitor_in_us = tim_us_last_af;
            //percent_af_adv = 100;
            //ignitor_in_us = tim_us_last_af; // set us delay from now(adv) to last adv-time, hit F roughly
            too_slow_rpm_use_f_fire = true; // fire on F please, avoid jitter of calculated from ADV

            percent_af_adv = 1;
            last_ignitor_in_us = ignitor_in_us; // stats
            if (use_tci)
            {
              tci_ignitor_dwell_in_us = 1; // start dwell
              // this is crude, we just start dwell at ADV signal here, to do better moving forward
              ignitor_engage();
            }
          }
          else if (tim_us_last_rev <= max_rpm_min_us) // faster RPM just give max advance
          {
            ignitor_in_us = 10; // set delay to fire almost immediately(at ADV mark here)
            percent_af_adv = 99;
            last_ignitor_in_us = ignitor_in_us; // stats
            if (use_tci)
            {
              tci_ignitor_dwell_in_us = 1; // start dwell
              //ignitor_in_us = 1;//CDI_TRIGGER_TIME; // best we can do at the moment
              ignitor_engage();
            }
          }
          else // between 2k and 6k rpm
          {
            if (use_tci)
            {
              tci_ignitor_dwell_in_us = 1; // start dwell
              ignitor_engage();
            }

            //percent_af_adv = (range_us - (tim_us_last_rev-max_rpm_min_us)) / af_us_div100;
            percent_af_adv = 100 - ((tim_us_last_rev-max_rpm_min_us) / af_us_div100);
            // example(@ 30ms=2krpm): 20k - (30000-10000) / 200 = 100 % 
            // example(@ 20ms=3krpm): 20k - (20000-10000) / 200 = 50 % 
            // example(@ 15ms=4krpm): 20k - (15000-10000) / 200 = 25 % 
            // example(@ 12ms=5krpm): 20k - (12000-10000) / 200 = 10 %
            // for now limit, but should not be needed.  Remove eventually.
            //  Also the 2 and 98 percent will be easy to see on printout that we are here scaling and near limit
            if (percent_af_adv > 98) percent_af_adv = 98;
            else if (percent_af_adv < 2) percent_af_adv = 2;
            uint16_t tim_us_calc_af = ((uint32_t)tim_us_last_rev * perc_x10_af_angle) / 1000U;
            //ignitor_in_us = (tim_us_last_af / 100) * (100 - percent_af_adv);
            ignitor_in_us = (tim_us_calc_af / 100) * (100 - percent_af_adv);
            last_ignitor_in_us = ignitor_in_us; // to show on stats
          }
        }
        else // advance not to be used yet, fire at F
        {
          percent_af_adv = 0;
          last_ignitor_in_us = 0;
          //if (use_tci) WIP!
          //{
          //  tci_ignitor_dwell_in_us = 1; // start dwell
          //  //ignitor_in_us = 
          //  // this is crude, we just start dwell at ADV signal here, to do better moving forward
          //  ignitor_engage();
          //}
        }
      }
    }
    if (ev & EV_F_MARK)
    {
      if (cnt_f_us_revbase > 4000) // ignore filter, us, after last F, crude and questionable
      {
        tim_us_last_af = cnt2_us_af_base;
        af_us_total_stats += tim_us_last_af;
        if (tim_us_last_af > af_us_max_stats)
          af_us_max_stats = tim_us_last_af;
        else if (tim_us_last_af < af_us_min_stats)
          af_us_min_stats = tim_us_last_af;

        if (is_only_f_used())
        { 
           tim_us_last_rev = cnt_f_us_revbase;
           if (cnt_revs_stats == (F_USED_CNT_NUM_40/2))
           {
             // store the percent ADV to F angle in percent based on this instance
             // TODO: base it on a average calculated on more than one.
             // This is critical measure and should be scrutinized.
             // This is used as a max ADV to F angle for advance timing later
             perc_x10_af_angle = (uint16_t)(((uint32_t)tim_us_last_af * 1000) / tim_us_last_rev);
             sprintf(logln, " store x10perc:%d", perc_x10_af_angle);
             log(logln);
           }
           cnt_revs_stats += 1;
        }
                
        rev_us_total_stats += cnt_f_us_revbase;
        if (cnt_f_us_revbase > rev_us_max_stats)
          rev_us_max_stats = cnt_f_us_revbase;
        else if (cnt_f_us_revbase < rev_us_min_stats)
          rev_us_min_stats = cnt_f_us_revbase;
        cnt_f_us_revbase = 0;

        if (!opt_gen)
          led2_disengage();
        if (is_only_f_used() || too_slow_rpm_use_f_fire) // do not use adv, so fire at F
        { 
          fire_ign = CDI_TRIGGER_TIME;
          ignitor_engage();
          too_slow_rpm_use_f_fire = 0;
          if (f_use_cnt > 0)
             f_use_cnt -= 1;
        }
      }
      else
      {
        rev_us_error_cnt += 1;
      }
    }
    ev = 0;
  }

  //if (fire_led0 > 0)
  //  led0_disengage_work();
  //if (fire_led1 > 0)
  //  led1_disengage_work();
  if (tci_ignitor_dwell_in_us)
  {
    tci_dwell_disengage_work();
  }
  if (fire_ign > 0)
    ignitor_disengage_work();
}

//-----------------------------------
void work_1ms_io(void)
{
  // try to print stats at a time when not much is going on
  //pr_u16("4swork", tim_us_last_rev-(tim_us_last_af*2));
  //if (perform_1s_stats_request)
  if (perform_1s_stats_request && cnt_f_us_revbase  < (tim_us_last_rev-(tim_us_last_af*2)))
  {
    if ((perform_1s_stats_request & 1) != 0) // 1 second work
    {
      work_1s();
    }
    if ((perform_1s_stats_request & 2) != 0) // 4 second work
    {
      work_4s();
    }
    perform_1s_stats_request = 0;
  }
}

//-----------------------------------
void work_10ms_io(void)
{
  in_sw1 = digitalRead(io_in_sw1);
  in_sw2 = digitalRead(io_in_sw2);
  if (last_in_sw1 != in_sw1)
  {
    last_in_sw1 = in_sw1;
    pr_u16("Sw1", in_sw1);
#ifdef DO_RAMLOG
    dump_ramlog();
#endif
  }
  if (last_in_sw2 != in_sw2)
  {
    last_in_sw2 = in_sw2;
    pr_u16("Sw2", in_sw2);
  }
}

//-----------------------------------
void timing_work(void)
{
  // The TM_MASK here is to use a shorter roll over to help force
  // any rollover issues by making it roll over faster
  tim_us_now = system_get_time() & TM_MASK;
  // Deal with roll over, calculate a difference in time from last and use that
  if (tim_us_now >= tm_last)
  {
    tim_us_diff = tim_us_now - tm_last;
  }
  else // rolled over
  {
    tim_us_diff = (TM_MASK - tm_last) + tim_us_now + 1;
  }
  tm_last = tim_us_now;

  if (did_pr_stats_ignore_is_diff_stats)
     did_pr_stats_ignore_is_diff_stats = 0;
#ifdef DO_JITTER_STATS
  else
  {
    uint16_t t16_diff = (uint16_t) tim_us_diff;
    if (t16_diff > max_tim_us_diff_stats)
      max_tim_us_diff_stats = t16_diff;
    else if (tim_us_diff < min_tim_us_diff_stats)
      min_tim_us_diff_stats = t16_diff;
    total_tim_us_diff_stats += t16_diff;
    total_tim_us_diff_count += 1;
  }
#endif
  if (cnt_f_us_revbase < 65000)
     cnt_f_us_revbase += tim_us_diff;

  if (cnt_adv_us_revbase < 65000)
     cnt_adv_us_revbase += tim_us_diff;

  if (cnt2_us_af_base < 65000)
     cnt2_us_af_base += tim_us_diff;

  cnt2_us_timebase += tim_us_diff;
  if (cnt2_us_timebase > 100)
  {
    cnt2_us_timebase -= 100;
    cnt1_100us_timebase += tim_us_diff;
  }

  cnt1_us_timebase += tim_us_diff;
  if (cnt1_us_timebase > 1000) // every ms work
  {
    cnt1_us_timebase -= 1000;
    ++cnt1_ms_timebase;

    work_1ms_io();

    if (cnt1_ms_timebase >= 10)
    {
       cnt1_ms_timebase = 0;
       work_10ms_io();

      ++cnt2_10ms_timebase;
      if (cnt2_10ms_timebase > 100) // every second work
      {
        cnt2_10ms_timebase = 0;
        system_soft_wdt_feed();
        perform_1s_stats_request |= 1; // request 1 second work
        ++cnt1_s_timebase;
        if ((cnt1_s_timebase & 0x3) == 2) // every 4 seconds
        {
#ifndef TARGET_X86
          if (opt_gen)
#endif
          {
            // Generator auto ramp between idle and 6k rpm for example.
            switch(gen_auto_ramp)  // off if set to 0, otherwise counter for below
            {
               case 3:  gen_100us_base = 350; break;
               case 5:  gen_100us_base = 300; break;
               case 7:  gen_100us_base = 250; break;
               case 9:  gen_100us_base = 200; break;
               case 11: gen_100us_base = 150; break;
               case 13: gen_100us_base = 400; break;
               default: ; break;
            }
            if (gen_auto_ramp >= 13)
              gen_auto_ramp = 1;
            else if (gen_auto_ramp > 0)
              gen_auto_ramp += 1;
          }
#ifndef TARGET_X86
          else // if not generator do 4 second work
#endif
            perform_1s_stats_request |= 2; // request 4 second work
        }
      }
    }
  }
}

//-----------------------------------
// Output on led2 a generated ADV mark, on io_out_ign2 a F mark.
void testgen_work(void)
{
  static uint16_t last_cnt2_100us = 0;
  static uint16_t gen_cnt_100us = 0;
  if (last_cnt2_100us != cnt1_100us_timebase)
  {
    last_cnt2_100us = cnt1_100us_timebase;
    ++gen_cnt_100us;
    #define width_100us_pulse 4
#ifdef TARGET_X86
    if (gen_cnt_100us == (gen_100us_base-(13*width_100us_pulse)))
      sim_io |= SIM_IO_A;
    else if (gen_cnt_100us == (gen_100us_base-(12*width_100us_pulse)))
      sim_io &= ~SIM_IO_A;
    else if (gen_cnt_100us == gen_100us_base-(1*width_100us_pulse))
      sim_io |= SIM_IO_F;
    else if (gen_cnt_100us >= gen_100us_base)
    {
      sim_io &= ~SIM_IO_F;
      gen_cnt_100us = 0;
    }
#else
    if (gen_cnt_100us == (gen_100us_base-(13*width_100us_pulse)))
      digitalWrite(io_led1, cfg_gen_lo ? 0 : 1);
    else if (gen_cnt_100us == (gen_100us_base-(12*width_100us_pulse)))
      digitalWrite(io_led1, cfg_gen_lo ? 1 : 0);
    else if (gen_cnt_100us == gen_100us_base-(1*width_100us_pulse))
      digitalWrite(io_out_ign2, cfg_gen_lo ? 0 : 1);
    else if (gen_cnt_100us >= gen_100us_base)
    {
      digitalWrite(io_out_ign2, cfg_gen_lo ? 1 : 0);
      gen_cnt_100us = 0;
    }
#endif
  }
}

//-----------------------------------
void prog_start(void)
{
  tm_last = system_get_time();

  //uint16_t io_pins = portInputRegister(0);
  in_sw1 = digitalRead(io_in_sw1);
  in_sw2 = digitalRead(io_in_sw2);
  last_in_sw1 = in_sw1;
  last_in_sw2 = in_sw2;
  opt_gen = is_sw2_on();
  uint16_t in_f = digitalRead(io_in_f);
  uint16_t in_a = digitalRead(io_in_a);

#ifdef DO_FS
  //use_fs = is_sw1_adv_on() && !opt_gen;
  use_fs = !opt_gen;
  if (use_fs)
  {
    if (!SPIFFS.begin())
      Serial.println("FS failed!");

    dump_log();
    Serial.println("--log dmp end---");
    if (is_sw1_adv_on())
    {
      write_log("clear log", 1);
      //if (!SPIFFS.format())
      //  Serial.println("failed spiffs frmt");
      //else
      //  Serial.println("spiffs frmt OK");
    }
  }
#endif

  //pr_u32("\n****** Start tm", tm_last);
  log("****");  
  sprintf(logln, "** Start - sw1:%d sw2:%d f:%d a:%d e:%d", last_in_sw1, last_in_sw2, in_f, in_a, use_f_edges);
  log(logln);


  digitalWrite(io_out_ign1, use_ign_pos ? 0 : 1);
  digitalWrite(io_out_ign2, use_ign_pos ? 0 : 1);
  //digitalWrite(io_led3, 1);
  digitalWrite(io_led1, use_led1_pos ? 0 : 1);
  digitalWrite(io_led0, 1);

// this one toggles ignitor fire at start, slower, more pulses to test on bench
//#define DO_STARTUP_FIRE_DANCE

// this one can be used at startup to show startup on LED's, short quick, no fire
#define DO_STARTUP_LED_WIGGLE
#ifdef DO_STARTUP_LED_WIGGLE
  // brief blink if LED outputs at start, for short start indicator
#ifdef DO_STARTUP_FIRE_DANCE
  for (int i=0; i<120; i++)
#else
  for (int i=0; i<10; i++)
#endif
  {
    //digitalWrite(io_led3, 0);
    digitalWrite(io_led1, use_led1_pos ? 1 : 0);
    digitalWrite(io_led0, 0);
#ifdef DO_STARTUP_FIRE_DANCE
    digitalWrite(io_out_ign1, use_ign_pos ? 1 : 0);
    digitalWrite(io_out_ign2, use_ign_pos ? 1 : 0);
    delayMicroseconds(250000); // 250ms
    //delayMicroseconds(3000); // 3ms, test fire TCI
#else
    delayMicroseconds(50000); // 50ms
#endif
    //digitalWrite(io_led3, 1);
    digitalWrite(io_led1, use_led1_pos ? 0 : 1);
    digitalWrite(io_led0, 1);
#ifdef DO_STARTUP_FIRE_DANCE
    digitalWrite(io_out_ign1, use_ign_pos ? 0 : 1);
    digitalWrite(io_out_ign2, use_ign_pos ? 0 : 1);
    delayMicroseconds(250000); // 250ms
#endif
    delayMicroseconds(50000); // 50ms
    system_soft_wdt_feed(); // Kick watchdog or it will timeout after a few seconds and restart
  }
#endif

  tm_last = system_get_time();
}

//-----------------------------------
void main_prog(void)
{
  prog_start();

  while (true)
  {
    ev_detect();
    timing_work();
#ifdef TARGET_X86
    // This generates a trace file on PC of simulated IO changes
    // We can look at that or generate scope snapshot views based on it to help test
    if (sim_io_last != sim_io)
    {
      static uint32_t sim_us_tm_last = 0;
      FILE *fp = fopen("sim_io_trace.txt", "a");
      if (fp)
      {
        char cmmt[64]; // comment added to notate for humans
        char tm_str[64]; // time and IO numbers for parsing

        uint32_t sim_us_delta = sim_us_tm - sim_us_tm_last;
        //sim_us_delta = sim_io - sim_io_last;
        cmmt[0] = (sim_io & SIM_IO_A) ? 'A' : '_';
        cmmt[1] = (sim_io & SIM_IO_F) ? 'F' : '_';
        cmmt[2] = (sim_io & SIM_IO_i1) ? '1' : '_';
        cmmt[3] = (sim_io & SIM_IO_L1) ? 'L' : '_';
        cmmt[4] = ' ';
        sprintf(&cmmt[5], "%5d", sim_us_delta);
        sprintf(tm_str, "%d %3d", sim_us_tm, sim_io);

        fprintf(fp, "%16s; %s\n", tm_str, cmmt);
         fclose(fp);
      }
      sim_io_last = sim_io;
      sim_us_tm_last = sim_us_tm;
    }
    sim_us_tm += 20; // bump sim time by 20 usecs
    if (sim_us_tm > 60000000) // done with simulation?
      return;
#else
    if (opt_gen) // act as generator of test signals
#endif
      testgen_work();
    ign_work();
  }
}

#ifdef TARGET_D1_MINI
//-----------------------------------
// this is an arduino construct, we just use as main entry point and take over looping
void loop(void)
{
  //system_soft_wdt_stop();
#ifdef DO_WIFI_SERVER
  server.handleClient();
#endif
  main_prog();
}
#endif

#ifdef TARGET_X86
void sim_x86_work()
{
}

int main()
{
  printf("start sim\n");
  sim_io |= SIM_IO_SW2; // turn sw1 on(use ADV), sw2 off(on when zero).
  main_prog();
  printf("end sim\n");
}
#endif
