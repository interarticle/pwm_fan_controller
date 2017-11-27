const int PWM_PIN = 25;
const int LED_PIN = 13;

const uint8_t PWM_INCREMENT = 20;
const uint8_t PWM_MAX = 200;
const int WAIT_CYCLES = 5;
const int MONITOR_CYCLES = 2;

enum op_mode_t {
  OP_MODE_WAIT,
  OP_MODE_START,
  OP_MODE_MONITOR,
};

uint8_t pwm_output;
uint8_t wait_count;
uint8_t monitor_count;
enum op_mode_t op_mode;

void setup_quad_dec() {
  // Port configuration
  PORTB_PCR0 = PORT_PCR_MUX(6) | PORT_PCR_PE | PORT_PCR_PS; // B0 = FTM1 QD PHA; PULLUP
  PORTB_PCR1 = PORT_PCR_MUX(6) | PORT_PCR_PE | PORT_PCR_PS; // B0 = FTM1 QD PHB

  // FTM Configuration
  
  FTM1_MODE = FTM_MODE_WPDIS;
  FTM1_SC = 0;  // Disable FTM1
  FTM1_CNTIN = 0;
  FTM1_CNT = 0;
  FTM1_MOD = 0xffff;
  FTM1_MODE |= FTM_MODE_FTMEN;

  FTM1_QDCTRL = FTM_QDCTRL_QUADEN | FTM_QDCTRL_QUADMODE;

  FTM1_SC = FTM_SC_CLKS(3);  // Enable FTM1
}

int read_clear_decoder() {
  int result = FTM1_CNT;
  FTM1_CNT = 0;
  return result;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 25000);
  pwm_output = 0;
  wait_count = 0;
  monitor_count = 0;
  op_mode = OP_MODE_WAIT;

  setup_quad_dec();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (op_mode == OP_MODE_WAIT) {
    pwm_output = 0;
  }
  analogWrite(PWM_PIN, pwm_output);

  if (op_mode == OP_MODE_START) {
    if (pwm_output < PWM_MAX) {
      if ((int) pwm_output + PWM_INCREMENT > PWM_MAX) {
        pwm_output = PWM_MAX;
      } else {
        pwm_output += PWM_INCREMENT;
      }
    } else {
      op_mode = OP_MODE_MONITOR;
    }
  }

  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);

  {
    int current_speed = read_clear_decoder();
    switch (op_mode) {
     case OP_MODE_START:
     case OP_MODE_MONITOR:
      wait_count = 0;
      if (current_speed == 0) {
        monitor_count ++;
      } else {
        monitor_count = 0;
      }
      
      if (monitor_count >= MONITOR_CYCLES) {
        op_mode = OP_MODE_WAIT;
      }
      break;
     case OP_MODE_WAIT:
      monitor_count = 0;
      if (current_speed > 0) {
        wait_count ++;
      } else {
        wait_count = 0;
      }

      if (wait_count >= WAIT_CYCLES) {
        op_mode = OP_MODE_START;
      }
      break;
    }
  }

  if (0) {
    Serial.print(op_mode, DEC);
    Serial.write(" ");
    Serial.print(wait_count, DEC);
    Serial.write(" ");
    Serial.print(monitor_count, DEC);
    Serial.write(" ");
    Serial.print(pwm_output, DEC);
    Serial.write("\n");
  }
}
