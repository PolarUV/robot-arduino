#include <UIPEthernet.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MS5837.h>

#define dt 0.05f

// ############################## Ethernet ##############################

const byte Mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
const IPAddress RobotIP(192, 168, 1, 1);
const IPAddress RemoteIP(192, 168, 1, 2);
constexpr uint16_t Port = 8888;

EthernetUDP UDP;

// ############################## Двигатели ##############################

Servo motors[6];

enum MotorForce {Fx, Fy, Fz, Mx, My, Mz};

constexpr int16_t MotorCoefficients[6][6] =
{
// Fx    Fy    Fz   Mx   My  Mz  
  {0,    0,   60, 50,  50, 0  }, // Двигатель 1
  {0,    100, 0,   0,   0,  50 }, // Двигатель 2
  {0,    0,   60, -50, 50, 0  }, // Двигатель 3
  {-100, 0,   0,   0,   0,  0  }, // Двигатель 4
  {0,    100, 0,   0,   0,  -50}, // Двигатель 5
  {0,    0,   100, 0,  -50, 0  }  // Двигатель 6
};

int16_t OldMotorCommands[6] = {0, 0, 0, 0, 0, 0};

// ############################## Датчик ориентации ##############################

Adafruit_BNO055 SensorBNO055 = Adafruit_BNO055(-1, 0x29);
float TargetOrientation[3] = {0.0f, 0.0f, 0.0f};
float OldErrorX = 0.0f;
float OldErrorY = 0.0f;
float OldErrorZ = 0.0f;
float IntegralX = 0.0f;
float IntegralY = 0.0f;
float IntegralZ = 0.0f;

enum Angle {X, Y, Z};

// ############################## Датчик глубины ##############################

MS5837 SensorMS5837;
float TargetDepth = 0.0f;
float OldErrorDepth = 0.0f;
float IntegralDepth = 0.0f;

// ############################## Стабилизация ##############################

enum Mode
{
  RemoteControl,
  Stabilization,
  Parking
};
uint8_t CurrentMode = Mode::RemoteControl;

struct RequestPacketStruct
{
  uint8_t Mode = Mode::RemoteControl;
  uint8_t LeftStickX = 127;
  uint8_t LeftStickY = 127;
  uint8_t RightStickX = 127;
  uint8_t RightStickY = 127;
  uint8_t ButtonL1 = 0;
  uint8_t ButtonL2 = 0;
  uint8_t ButtonR1 = 0;
  uint8_t ButtonR2 = 0;
};

struct ResponsePacketStruct
{
  uint8_t state;
};

// ####################################################################################################
// #                                             Setup                                                #
// ####################################################################################################

void setup()
{
  // Запуск интерфейса I2C
  Wire.begin();

  // Открытие последовательного порта
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Setup ... ");

  // Выбор номера вывода CS Ethernet-контроллера
  Ethernet.init(41);

  // Запуск Ethernet-контроллера
  Ethernet.begin(Mac, RobotIP);

  // Проверка статуса подключения Ethernet-контроллера
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("ERROR - Can't find Ethernet controller");
    while(1);
  }

  // Запуск UDP
  UDP.begin(Port);

  // Инициализация датчика ориентации
  if (!SensorBNO055.begin())
  {
    Serial.println("ERROR - Can't find orientation sensor");
    while(1);
  }

  // Инициализация датчика глубины
  if (!SensorMS5837.init())
  {
    Serial.println("ERROR - Can't find depth sensor");
    while(1);
  }
  SensorMS5837.setModel(MS5837::MS5837_30BA);
  SensorMS5837.setFluidDensity(997);

  // Настройка двигателей
  uint16_t motorPin = 11;
  for (auto motor : motors)
  {
    motor.attach(motorPin++);
    motor.writeMicroseconds(1500);
    motor.writeMicroseconds(1500);
    motor.writeMicroseconds(1500);
  }
  delay(3000);

  Serial.println("OK");
}

// ####################################################################################################
// #                                               Loop                                               #
// ####################################################################################################

void loop() {
  // Если есть входящий пакет
  uint16_t packetSize = UDP.parsePacket();
  if (packetSize)
  {
    // Создание структуры пакета
    RequestPacketStruct requestPacket;

    // Прием пакета
    UDP.read((char*)&requestPacket, sizeof(RequestPacketStruct));

    int16_t forceFx;
    int16_t forceFy;
    int16_t forceFz;
    int16_t forceMx;
    int16_t forceMy;
    int16_t forceMz;
    int16_t motorCommands[6];

    // Расшифровка пакета
    uint8_t mode = requestPacket.Mode;
    // Режим дистанционного управления
    if (mode == Mode::RemoteControl)
    {
      CurrentMode = Mode::RemoteControl;
        
      // Перевод состояния геймпада в векторы сил
      forceFx = map(requestPacket.RightStickY, 0, 255, -500, 500);
      forceFy = map(requestPacket.LeftStickX, 0, 255, -500, 500);
      forceFz = map(requestPacket.ButtonR2, 0, 255, 0, 500) * (requestPacket.ButtonR1 == 0 ? 1 : -1);
      forceMx = map(requestPacket.ButtonL2, 0, 255, 0, 500) * (requestPacket.ButtonL1 == 0 ? 1 : -1);
      forceMy = map(requestPacket.LeftStickY, 0, 255, -500, 500);
      forceMz = map(requestPacket.RightStickX, 0, 255, -500, 500);
    }
    // Режим стабилизации и парковки
    else
    {
      // Считывание текущей ориентации
      sensors_event_t event;
      SensorBNO055.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
      float currentOrientation[3] =
      {
        event.orientation.z + 180.0f, // [-180; 180] -> [0; 360] (Перепутано с осью X)
        event.orientation.y + 90.0f,  // [-90; 90]   -> [0; 180]
        event.orientation.x           //                [0; 360] (Перепутано с осью Z)
      };

      // Считывание текущей глубины
      SensorMS5837.read();
      float currentDepth = SensorMS5837.depth();

      // Если режим стабилизации был только что включен
      if (mode == Mode::Stabilization && CurrentMode != Mode::Stabilization)
      {
        // Обнуление интегральных составляющих
        IntegralX = 0.0f;
        IntegralY = 0.0f;
        IntegralZ = 0.0f;
        IntegralDepth = 0.0f;
        // Задание цели стабилизации
        for (uint8_t i = 0; i < 3; i++)
        {
          TargetOrientation[i] = currentOrientation[i];
        }
        TargetDepth = currentDepth;
        CurrentMode = Mode::Stabilization;
      }
      // Если режим парковки был только что включен
      else if (mode == Mode::Parking && CurrentMode != Mode::Parking)
      {
        // Обнуление интегральных составляющих
        IntegralX = 0.0f;
        IntegralY = 0.0f;
        IntegralZ = 0.0f;
        IntegralDepth = 0.0f;
        // Задание цели стабилизации
        TargetOrientation[Angle::X] = 180.0f;
        TargetOrientation[Angle::Y] = 90.0f;
        TargetOrientation[Angle::Z] = currentOrientation[Angle::Z];
        TargetDepth = 0.3f;
        CurrentMode = Mode::Parking;
      }

      // Расчет текущих ошибок
      float currentErrorX = currentOrientation[Angle::X] - TargetOrientation[Angle::X];
      if (currentErrorX > 180.0f) currentErrorX = 360.0f - currentErrorX; 
      float currentErrorY = currentOrientation[Angle::Y] - TargetOrientation[Angle::Y];
      if (currentErrorY > 90.0f) currentErrorY = 180.0f - currentErrorY;
      float currentErrorZ = currentOrientation[Angle::Z] - TargetOrientation[Angle::Z];
      if (currentErrorZ > 180.0f) currentErrorZ = 360.0f - currentErrorZ;
      float currentErrorDepth = currentDepth - TargetDepth;

      // Расчет значения ПИД-регулятора оси Х
      float proportionalX = currentErrorX;
      IntegralX = IntegralX + currentErrorX * dt;
      float differentialX = (currentErrorX - OldErrorX) / dt;
      int16_t orientationPIDX = 10.0f * proportionalX + 2.0f * IntegralX + 1.0f * differentialX;

      // Расчет значения ПИД-регулятора оси Y
      float proportionalY = currentErrorY;
      IntegralY = IntegralY + currentErrorY * dt;
      float differentialY = (currentErrorY - OldErrorY) / dt;
      int16_t orientationPIDY = 15.0f * proportionalY + 3.0f * IntegralY + 1.5f * differentialY;

      // Расчет значения ПИД-регулятора оси Z
      float proportionalZ = currentErrorZ;
      IntegralZ = IntegralZ + currentErrorZ * dt;
      float differentialZ = (currentErrorZ - OldErrorZ) / dt;
      int16_t orientationPIDZ = 5.0f * proportionalZ + 1.0f * IntegralZ + 0.5f * differentialZ;

      // Расчет значения ПИД-регулятора глубины
      float proportionalDepth = currentErrorDepth;
      IntegralDepth = IntegralDepth + currentErrorDepth * dt;
      float differentialDepth = (currentErrorDepth - OldErrorDepth) / dt;
      int16_t depthPID = 600.0f * proportionalDepth + 120.0f * IntegralDepth + 60.0f * differentialDepth;

      // Расчет векторов сил
      forceFx = depthPID * sin(radians(currentOrientation[Angle::Y]-90.0f));
      forceFy = -depthPID * sin(radians(currentOrientation[Angle::X]-180.0f)) * cos(radians(currentOrientation[Angle::X]-180.0f));
      forceFz = -depthPID * cos(radians(currentOrientation[Angle::X]-180.0f)) * cos(radians(currentOrientation[Angle::Y]-90.0f));
      forceMx = -orientationPIDX;
      forceMy = orientationPIDY;
      forceMz = -orientationPIDZ;
    }

    //Расчет команд для двигателей
    for (uint8_t i = 0; i < 6; i++)
    {
      motorCommands[i] =
        (int16_t) (forceFx * (float)MotorCoefficients[i][MotorForce::Fx] / 100.0f) +
        (int16_t) (forceFy * (float)MotorCoefficients[i][MotorForce::Fy] / 100.0f) +
        (int16_t) (forceFz * (float)MotorCoefficients[i][MotorForce::Fz] / 100.0f) +
        (int16_t) (forceMx * (float)MotorCoefficients[i][MotorForce::Mx] / 100.0f) +
        (int16_t) (forceMy * (float)MotorCoefficients[i][MotorForce::My] / 100.0f) +
        (int16_t) (forceMz * (float)MotorCoefficients[i][MotorForce::Mz] / 100.0f);
    }

    // Масштабирование значений команд до диапазона [-500; 500]
    int16_t maxCommand = 0;
    for (auto command : motorCommands)
    {
      if (abs(command) > maxCommand) maxCommand = abs(command);
    }
    if (maxCommand > 500)
    {
      float scalingFactor = 500.0f / maxCommand;
      for (uint8_t i = 0; i < 6; i++)
      {
        motorCommands[i] *= scalingFactor;
      }
    }
        
    // Сглаживание изменения значений команд // ToDo: сделать масштабирование в относительных величинах
    for (uint8_t i = 0; i < 6; i++)
    {
      if ((motorCommands[i] - OldMotorCommands[i]) > 25)
        motorCommands[i] = OldMotorCommands[i] + 25;
      else if ((motorCommands[i] - OldMotorCommands[i]) < -25)
        motorCommands[i] = OldMotorCommands[i] - 25;
    }

    // Отправка команд на двигатели
    //Serial.print("[ ");
    for (uint8_t i = 0; i < 6; i++)
    {
      int16_t command = 1500 + motorCommands[i];
      motors[i].writeMicroseconds(command);
      //Serial.print(command);
      //Serial.print(" ");
    }
    //Serial.println("]");

    // Сохранение команд двигателей
    for (uint8_t i = 0; i < 6; i++)
    {
      OldMotorCommands[i] = motorCommands[i];
    }

    // Отправка ответа на пульт управления
    switch (mode)
    {
      case Mode::RemoteControl:
      {
        const char replyBuffer = 'R';
        UDP.beginPacket(RemoteIP, Port);
        UDP.write(&replyBuffer);
        UDP.endPacket();
        break;
      }
      case Mode::Stabilization:
      {
        const char replyBuffer = 'S';
        UDP.beginPacket(RemoteIP, Port);
        UDP.write(&replyBuffer);
        UDP.endPacket();
        break;
      }
      case Mode::Parking:
      {
        const char replyBuffer = 'P';
        UDP.beginPacket(RemoteIP, Port);
        UDP.write(&replyBuffer);
        UDP.endPacket();
        break;
      }
    }    
  }
}
