#include "QTR8A.h"
#include <stdlib.h>
#include <driver/gpio.h>
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"    // Para tareas críticas
#include "esp_intr_alloc.h"   // Para interrupciones si las necesitas (opcional)
#include <vector>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc_cal.h"


adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_oneshot_unit_handle_t adc2_handle = NULL;
esp_adc_cal_characteristics_t *adc1_chars = NULL;
esp_adc_cal_characteristics_t *adc2_chars = NULL;

const adc_channel_t adc_channels[8] = {
    ADC_CHANNEL_0,  // Canal 0 (GPIO36 en ADC1)
    ADC_CHANNEL_3,  // Canal 3 (GPIO39 en ADC1)
    ADC_CHANNEL_6,  // Canal 6 (GPIO34 en ADC1)
    ADC_CHANNEL_7,  // Canal 7 (GPIO35 en ADC1)
    ADC_CHANNEL_4,  // Canal 4 (GPIO32 en ADC1)
    ADC_CHANNEL_5,  // Canal 5 (GPIO33 en ADC1)
    ADC_CHANNEL_8,  // Canal 8 (GPIO25 en ADC2)
    ADC_CHANNEL_9   // Canal 9 (GPIO26 en ADC2)
};

adc_channel_t batt_channel = ADC_CHANNEL_5; // GPIO12

void QTR8A::setTypeRC()
{
  _type = QTRType::RC;
  _maxValue = _timeout;
}

void QTR8A::setTypeAnalog()
{
  _type = QTRType::Analog;
  _maxValue = 4095; // 1023; // Arduino analogRead() returns a 10-bit value by default
}

void QTR8A::setSensorPins(const std::vector<gpio_num_t>& pins, uint8_t sensorCount, uint16_t threshold)
{
  _threshold = threshold;
  if (sensorCount > QTRMaxSensors) { sensorCount = QTRMaxSensors; }

  // (Re)allocate and initialize the array if necessary.
  uint8_t * oldSensorPins = _sensorPins;
  _sensorPins = (uint8_t *)realloc(_sensorPins, sizeof(uint8_t) * sensorCount);
  if (_sensorPins == nullptr)
  {
    // Memory allocation failed; don't continue.
    free(oldSensorPins); // deallocate any memory used by old array
    return;
  }

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    _sensorPins[i] = pins[i];
  }

  _sensorPins[0] = ADC_CHANNEL_0;
  _sensorPins[1] = ADC_CHANNEL_3;
  _sensorPins[2] = ADC_CHANNEL_6;
  _sensorPins[3] = ADC_CHANNEL_7;
  _sensorPins[4] = ADC_CHANNEL_4;
  _sensorPins[5] = ADC_CHANNEL_5;

  _sensorPins[6] = ADC_CHANNEL_8;
  _sensorPins[7] = ADC_CHANNEL_9;


  
  // Configurar ADC1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    //adc_oneshot_new_unit(&init_config1, &adc1_handle);
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configurar ADC2
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    // 4. Configurar cada canal individualmente
    for (int i = 0; i < 8; i++) {
        if (adc_channels[i] <= ADC_CHANNEL_7) {
            // Canales 0-7 en ADC1
            ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channels[i], &config));
        } else {
            // Canales 8-9 en ADC2
            ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, adc_channels[i], &config));
        }
    }

    //CONFIGURA EL CANAL PARA LA LECTURA DE LA BATERIA
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle,batt_channel , &config));


    adc1_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc2_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, (adc_bits_width_t)ADC_BITWIDTH_12, 1100, adc1_chars);
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_12, (adc_bits_width_t)ADC_BITWIDTH_12, 1100, adc2_chars);

  _sensorCount = sensorCount;

  // Any previous calibration values are no longer valid, and the calibration
  // arrays might need to be reallocated if the sensor count was changed.
  calibrationOn.initialized = false;
  calibrationOff.initialized = false;
}

int QTR8A::readBatt(void){
    int raw_value = 0;
    // Leer el valor del ADC2 en el canal de la batería
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, batt_channel, &raw_value));
    return raw_value; // Devuelve el voltaje en mV
}

void QTR8A::setTimeout(uint16_t timeout)
{
  if (timeout > 32767) { timeout = 32767; }
  _timeout = timeout;
  if (_type == QTRType::RC) { _maxValue = timeout; }
}

void QTR8A::setSamplesPerSensor(uint8_t samples)
{
  if (samples > 64) { samples = 64; }
  _samplesPerSensor = samples;
}

void QTR8A::setEmitterPin(uint8_t emitterPin)
{
    releaseEmitterPins();

    _oddEmitterPin = emitterPin;

    gpio_reset_pin((gpio_num_t)emitterPin); // Limpia configuración previa
    gpio_set_direction((gpio_num_t)emitterPin, GPIO_MODE_OUTPUT);

    _emitterPinCount = 1;
}

void QTR8A::setEmitterPins(uint8_t oddEmitterPin, uint8_t evenEmitterPin)
{
  releaseEmitterPins();

  _oddEmitterPin = oddEmitterPin;
  _evenEmitterPin = evenEmitterPin;
  gpio_set_direction((gpio_num_t)_oddEmitterPin, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)_evenEmitterPin, GPIO_MODE_OUTPUT);
  _emitterPinCount = 2;
}

void QTR8A::releaseEmitterPins()
{
  if (_oddEmitterPin != QTRNoEmitterPin)
  {
    gpio_set_direction((gpio_num_t)_oddEmitterPin, GPIO_MODE_INPUT);
    _oddEmitterPin = QTRNoEmitterPin;
  }

  if (_evenEmitterPin != QTRNoEmitterPin)
  {
    gpio_set_direction((gpio_num_t)_evenEmitterPin, GPIO_MODE_INPUT);
    _evenEmitterPin = QTRNoEmitterPin;
  }

  _emitterPinCount = 0;
}

void QTR8A::setDimmingLevel(uint8_t dimmingLevel)
{
  if (dimmingLevel > 31) { dimmingLevel = 31; }
  _dimmingLevel = dimmingLevel;
}

// emitters defaults to QTREmitters::All; wait defaults to true
void QTR8A::emittersOff(QTREmitters emitters, bool wait)
{
  bool pinChanged = false;

  // Use odd emitter pin in these cases:
  // - 1 emitter pin, emitters = all
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = odd
  if (emitters == QTREmitters::All ||
      (_emitterPinCount == 2 && emitters == QTREmitters::Odd))
  {
    // Check if pin is defined and only turn off if not already off
    if ((_oddEmitterPin != QTRNoEmitterPin) &&
        gpio_get_level((gpio_num_t)_oddEmitterPin) == 1)
    {
      gpio_set_level((gpio_num_t)_oddEmitterPin, 0);
      pinChanged = true;
    }
  }

  // Use even emitter pin in these cases:
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = even
  if (_emitterPinCount == 2 &&
      (emitters == QTREmitters::All || emitters == QTREmitters::Even))
  {
    // Check if pin is defined and only turn off if not already off
    if ((_evenEmitterPin != QTRNoEmitterPin) &&
        gpio_get_level((gpio_num_t)_evenEmitterPin) == 1)
    {
      gpio_set_level((gpio_num_t)_evenEmitterPin, 0);
      pinChanged = true;
    }
  }

  if (wait && pinChanged)
  {
    if (_dimmable)
    {
      esp_rom_delay_us(1200);
    }
    else
    {
      esp_rom_delay_us(200);
    }
  }
}

void QTR8A::emittersOn(QTREmitters emitters, bool wait)
{
  bool pinChanged = false;
  uint16_t emittersOnStart;

  // Use odd emitter pin in these cases:
  // - 1 emitter pin, emitters = all
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = odd
  if (emitters == QTREmitters::All ||
      (_emitterPinCount == 2 && emitters == QTREmitters::Odd))
  {
    // Check if pin is defined, and only turn on non-dimmable sensors if not
    // already on, but always turn dimmable sensors off and back on because
    // we might be changing the dimming level (emittersOnWithPin() should take
    // care of this)
    if ((_oddEmitterPin != QTRNoEmitterPin) &&
        ( _dimmable || gpio_get_level((gpio_num_t)_oddEmitterPin) == 0))
    {
      emittersOnStart = emittersOnWithPin(_oddEmitterPin);
      pinChanged = true;
    }
  }

  // Use even emitter pin in these cases:
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = even
  if (_emitterPinCount == 2 &&
      (emitters == QTREmitters::All || emitters == QTREmitters::Even))
  {
    // Check if pin is defined, and only turn on non-dimmable sensors if not
    // already on, but always turn dimmable sensors off and back on because
    // we might be changing the dimming level (emittersOnWithPin() should take
    // care of this)
    if ((_evenEmitterPin != QTRNoEmitterPin) &&
        (_dimmable || gpio_get_level((gpio_num_t)_evenEmitterPin) == 0))
    {
      emittersOnStart = emittersOnWithPin(_evenEmitterPin);
      pinChanged = true;
    }
  }

   if (wait && pinChanged)
  {
    if (_dimmable)
    {
      while ((esp_timer_get_time() - emittersOnStart) < 300)
      {
        esp_rom_delay_us(10);
      }
    }
    else
    {
      esp_rom_delay_us(200);
    }
  }
}

// assumes pin is valid (not QTRNoEmitterPin)
// returns time when pin was first set high (used by emittersSelect())

// Define un mutex estático (al inicio del archivo o como miembro estático de clase)
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint16_t QTR8A::emittersOnWithPin(uint8_t pin)
{
    gpio_num_t gpioPin = (gpio_num_t)pin;

    if (_dimmable && gpio_get_level(gpioPin) == 1)
    {
        gpio_set_level(gpioPin, 0);
        esp_rom_delay_us(1200);
    }

    gpio_set_level(gpioPin, 1);
    uint16_t emittersOnStart = (uint16_t)esp_timer_get_time();

    if (_dimmable && (_dimmingLevel > 0))
    {
        taskENTER_CRITICAL(&mux);

        for (uint8_t i = 0; i < _dimmingLevel; i++)
        {
            esp_rom_delay_us(1);
            gpio_set_level(gpioPin, 0);
            esp_rom_delay_us(1);
            gpio_set_level(gpioPin, 1);
        }

        taskEXIT_CRITICAL(&mux);
    }

    return emittersOnStart;
}


void QTR8A::emittersSelect(QTREmitters emitters)
{
    QTREmitters offEmitters;

    switch (emitters)
    {
        case QTREmitters::Odd:
            offEmitters = QTREmitters::Even;
            break;

        case QTREmitters::Even:
            offEmitters = QTREmitters::Odd;
            break;

        case QTREmitters::All:
            emittersOn();
            return;

        case QTREmitters::None:
            emittersOff();
            return;

        default:
            return;
    }

    // Apaga los emisores contrarios sin esperar
    emittersOff(offEmitters, false);
    uint64_t turnOffStart = esp_timer_get_time();  // tiempo en microsegundos

    // Enciende los emisores deseados y espera
    emittersOn(emitters);

    if (_dimmable)
    {
        // Espera para asegurar que han pasado al menos 1200 us desde que se apagaron los otros emisores
        while ((esp_timer_get_time() - turnOffStart) < 1200)
        {
            esp_rom_delay_us(10);
        }
    }
}


void QTR8A::resetCalibration()
{
  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    if (calibrationOn.maximum)   { calibrationOn.maximum[i] = 0; }
    if (calibrationOff.maximum)  { calibrationOff.maximum[i] = 0; }
    if (calibrationOn.minimum)   { calibrationOn.minimum[i] = _maxValue; }
    if (calibrationOff.minimum)  { calibrationOff.minimum[i] = _maxValue; }
  }
}

void QTR8A::calibrate(QTRReadMode mode)
{
  // manual emitter control is not supported
  if (mode == QTRReadMode::Manual) { return; }

  if (mode == QTRReadMode::On ||
      mode == QTRReadMode::OnAndOff)
  {
    calibrateOnOrOff(calibrationOn, QTRReadMode::On);
  }
  else if (mode == QTRReadMode::OddEven ||
           mode == QTRReadMode::OddEvenAndOff)
  {
    calibrateOnOrOff(calibrationOn, QTRReadMode::OddEven);
  }

  if (mode == QTRReadMode::OnAndOff ||
      mode == QTRReadMode::OddEvenAndOff ||
      mode == QTRReadMode::Off)
  {
    calibrateOnOrOff(calibrationOff, QTRReadMode::Off);
  }
}

void QTR8A::calibrateOnOrOff(CalibrationData & calibration, QTRReadMode mode)
{
  uint16_t sensorValues[QTRMaxSensors];
  uint16_t maxSensorValues[QTRMaxSensors];
  uint16_t minSensorValues[QTRMaxSensors];

  // (Re)allocate and initialize the arrays if necessary.
  if (!calibration.initialized)
  {
    uint16_t * oldMaximum = calibration.maximum;
    calibration.maximum = (uint16_t *)realloc(calibration.maximum,
                                              sizeof(uint16_t) * _sensorCount);
    if (calibration.maximum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMaximum); // deallocate any memory used by old array
      return;
    }

    uint16_t * oldMinimum = calibration.minimum;
    calibration.minimum = (uint16_t *)realloc(calibration.minimum,
                                              sizeof(uint16_t) * _sensorCount);
    if (calibration.minimum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMinimum); // deallocate any memory used by old array
      return;
    }

    // Initialize the max and min calibrated values to values that
    // will cause the first reading to update them.
    for (uint8_t i = 0; i < _sensorCount; i++)
    {
      calibration.maximum[i] = 0;
      calibration.minimum[i] = _maxValue;
    }

    calibration.initialized = true;
  }

  for (uint8_t j = 0; j < 10; j++)
  {
    read(sensorValues, mode);

    for (uint8_t i = 0; i < _sensorCount; i++)
    {
      // set the max we found THIS time
      if ((j == 0) || (sensorValues[i] > maxSensorValues[i]))
      {
        maxSensorValues[i] = sensorValues[i];
      }

      // set the min we found THIS time
      if ((j == 0) || (sensorValues[i] < minSensorValues[i]))
      {
        minSensorValues[i] = sensorValues[i];
      }
    }
  }

  // record the min and max calibration values
  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    // Update maximum only if the min of 10 readings was still higher than it
    // (we got 10 readings in a row higher than the existing maximum).
    if (minSensorValues[i] > calibration.maximum[i])
    {
      calibration.maximum[i] = minSensorValues[i];
    }

    // Update minimum only if the max of 10 readings was still lower than it
    // (we got 10 readings in a row lower than the existing minimum).
    if (maxSensorValues[i] < calibration.minimum[i])
    {
      calibration.minimum[i] = maxSensorValues[i];
    }
  }
}

void QTR8A::read(uint16_t * sensorValues, QTRReadMode mode)
{
  switch (mode)
  {
    case QTRReadMode::Off:
      emittersOff();
      // fall through
    case QTRReadMode::Manual:
      readPrivate(sensorValues);
      return;

    case QTRReadMode::On:
    case QTRReadMode::OnAndOff:
      emittersOn();
      readPrivate(sensorValues);
      emittersOff();
      break;

    case QTRReadMode::OddEven:
    case QTRReadMode::OddEvenAndOff:
      // Turn on odd emitters and read the odd-numbered sensors.
      // (readPrivate takes a 0-based array index, so start = 0 to start with
      // the first sensor)
      emittersSelect(QTREmitters::Odd);
      readPrivate(sensorValues, 0, 2);

      // Turn on even emitters and read the even-numbered sensors.
      // (readPrivate takes a 0-based array index, so start = 1 to start with
      // the second sensor)
      emittersSelect(QTREmitters::Even);
      readPrivate(sensorValues, 1, 2);

      emittersOff();
      break;

    default: // invalid - do nothing
      return;
  }

  if (mode == QTRReadMode::OnAndOff ||
      mode == QTRReadMode::OddEvenAndOff)
  {
    // Take a second set of readings and return the values (on + max - off).

    uint16_t offValues[QTRMaxSensors];
    readPrivate(offValues);

    for (uint8_t i = 0; i < _sensorCount; i++)
    {
      sensorValues[i] += _maxValue - offValues[i];
      if (sensorValues[i] > _maxValue)
      {
        // This usually doesn't happen, because the sensor reading should
        // go up when the emitters are turned off.
        sensorValues[i] = _maxValue;
      }
    }
  }
}

void QTR8A::readCalibrated(uint16_t * sensorValues, QTRReadMode mode)
{
  // manual emitter control is not supported
  if (mode == QTRReadMode::Manual) { return; }

  // if not calibrated, do nothing

  if (mode == QTRReadMode::On ||
      mode == QTRReadMode::OnAndOff ||
      mode == QTRReadMode::OddEvenAndOff)
  {
    if (!calibrationOn.initialized)
    {
      return;
    }
  }

  if (mode == QTRReadMode::Off ||
      mode == QTRReadMode::OnAndOff ||
      mode == QTRReadMode::OddEvenAndOff)
  {
    if (!calibrationOff.initialized)
    {
      return;
    }
  }

  // read the needed values
  read(sensorValues, mode);
  
  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    uint16_t calmin, calmax;

    // find the correct calibration
    if (mode == QTRReadMode::On ||
        mode == QTRReadMode::OddEven)
    {
      calmax = calibrationOn.maximum[i];
      calmin = calibrationOn.minimum[i];
    }
    else if (mode == QTRReadMode::Off)
    {
      calmax = calibrationOff.maximum[i];
      calmin = calibrationOff.minimum[i];
    }
    else // QTRReadMode::OnAndOff, QTRReadMode::OddEvenAndOff
    {
      if (calibrationOff.minimum[i] < calibrationOn.minimum[i])
      {
        // no meaningful signal
        calmin = _maxValue;
      }
      else
      {
        // this won't go past _maxValue
        calmin = calibrationOn.minimum[i] + _maxValue - calibrationOff.minimum[i];
      }

      if (calibrationOff.maximum[i] < calibrationOn.maximum[i])
      {
        // no meaningful signal
        calmax = _maxValue;
      }
      else
      {
        // this won't go past _maxValue
        calmax = calibrationOn.maximum[i] + _maxValue - calibrationOff.maximum[i];
      }
    }

    uint16_t denominator = calmax - calmin;
    int16_t value = 0;

  
    if (denominator != 0)
    {
      value = (((int32_t)sensorValues[i]) - calmin) * 1000 / denominator;
    }
    
    if (value < _threshold) { value = 0; } //Modifique el valor de 0 a 500 para indicar si existe linea o no
    else if (value > 1000) { value = 1000; }
    
    sensorValues[i] = value;
  }
  
}



void QTR8A::readPrivate(uint16_t * sensorValues, uint8_t start, uint8_t step)
{
  int raw_value = 0;
    if (_sensorPins == nullptr) return;

    switch (_type)
    {
        case QTRType::RC:
            for (uint8_t i = start; i < _sensorCount; i += step)
            {
                sensorValues[i] = _maxValue;

                gpio_num_t pin = (gpio_num_t)_sensorPins[i];

                // Configura como salida y pone en HIGH
                gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                gpio_set_level(pin, 1);
            }

            esp_rom_delay_us(10); // carga las líneas durante 10 us

            {
                uint64_t startTime = esp_timer_get_time();
                uint16_t time = 0;

                // Configura todos los pines como entrada
                for (uint8_t i = start; i < _sensorCount; i += step)
                {
                    gpio_num_t pin = (gpio_num_t)_sensorPins[i];
                    gpio_set_direction(pin, GPIO_MODE_INPUT);
                }

                while (time < _maxValue)
                {
                    time = (uint16_t)(esp_timer_get_time() - startTime);

                    for (uint8_t i = start; i < _sensorCount; i += step)
                    {
                        gpio_num_t pin = (gpio_num_t)_sensorPins[i];
                        if ((gpio_get_level(pin) == 0) && (time < sensorValues[i]))
                        {
                            sensorValues[i] = time;
                        }
                    }
                }
            }
            return;

        case QTRType::Analog:
            for (uint8_t i = start; i < _sensorCount; i += step)
            {
                sensorValues[i] = 0;
            }

            for (uint8_t j = 0; j < _samplesPerSensor; j++)
            {
                for (uint8_t i = start; i < _sensorCount; i += step)
                {

                  if (adc_channels[i] <= ADC_CHANNEL_7)
                  {
                    // Lectura de ADC1
                    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channels[i], &raw_value));
                  }
                  else
                  {
                    // Lectura de ADC2
                    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, adc_channels[i], &raw_value));
                  }
                  sensorValues[i] += raw_value;
                }
            }
            
            
            for (uint8_t i = start; i < _sensorCount; i += step)
            {
                sensorValues[i] = (sensorValues[i] + (_samplesPerSensor >> 1)) / _samplesPerSensor;
                
            }
            
            return;

        default:
            return;
    }
}

uint16_t QTR8A::readLinePrivate(uint16_t * sensorValues, QTRReadMode mode,
                         bool invertReadings)
{
  bool onLine = false;
  uint32_t avg = 0; // this is for the weighted total
  uint16_t sum = 0; // this is for the denominator, which is <= 64000

  // manual emitter control is not supported
  if (mode == QTRReadMode::Manual) { return 0; }

  readCalibrated(sensorValues, mode);

  
  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    uint16_t value = sensorValues[i];
    if (invertReadings) { value = 1000 - value; } //linea negra es falso

    // keep track of whether we see the line at all
    if (value > 200) { onLine = true; } //el valor por default es 200

    
    // only average in values that are above a noise threshold
    if (value > 50)  //valor default 50
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }
  

  if (!onLine)
  {
    // If it last read to the left of center, return 0.
    if (_lastPosition < (_sensorCount - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (_sensorCount - 1) * 1000;
    }
  }

  _lastPosition = avg / sum;
  //printf("%lu %u %d\n", avg, sum, _lastPosition);
  return _lastPosition;
}

// the destructor frees up allocated memory
QTR8A::~QTR8A()
{
  releaseEmitterPins();

  if (_sensorPins)            { free(_sensorPins); }
  if (calibrationOn.maximum)  { free(calibrationOn.maximum); }
  if (calibrationOff.maximum) { free(calibrationOff.maximum); }
  if (calibrationOn.minimum)  { free(calibrationOn.minimum); }
  if (calibrationOff.minimum) { free(calibrationOff.minimum); }
}