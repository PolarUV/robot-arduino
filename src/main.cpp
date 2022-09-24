#include <UIPEthernet.h>
#include <Servo.h>

// ############################## Ethernet ##############################

const byte Mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
const IPAddress RobotIP(192, 168, 1, 1);
const IPAddress RemoteIP(192, 168, 1, 2);
constexpr uint16_t Port = 8888;

EthernetUDP UDP;

// ############################## Двигатели ##############################

Servo motors[6];

enum MotorForce {Fx, Fy, Fz, Mx, My, Mz};

int16_t MotorCoefficients[6][6]{};
bool MotorCoefficientsReceived = false;

int16_t OldMotorCommands[6]{};

struct RequestPacketStruct
{
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
  if (packetSize == sizeof(MotorCoefficients) && !MotorCoefficientsReceived)
  {
    Serial.println("Получены настройки");

    // Прием пакета
    UDP.read((char*)&MotorCoefficients, sizeof(MotorCoefficients));

    // Установка флага
    MotorCoefficientsReceived = true;

    // Отправка ответа на пульт управления
    const char replyBuffer = 'S'; // S = Setup
    UDP.beginPacket(RemoteIP, Port);
    UDP.write(&replyBuffer);
    UDP.endPacket(); 
  }
  else if (packetSize == sizeof(RequestPacketStruct))
  {
    Serial.println("Получены команды");

    // Создание структуры пакета
    RequestPacketStruct requestPacket;

    // Прием пакета
    UDP.read((char*)&requestPacket, sizeof(RequestPacketStruct));

    // Перевод состояния геймпада в векторы сил
    int16_t motorForces[6]{};
    motorForces[MotorForce::Fx] = map(requestPacket.RightStickY, 0, 255, -500, 500);
    motorForces[MotorForce::Fy] = map(requestPacket.LeftStickX, 0, 255, -500, 500);
    motorForces[MotorForce::Fz] = map(requestPacket.ButtonR2, 0, 255, 0, 500) * (requestPacket.ButtonR1 == 0 ? 1 : -1);
    motorForces[MotorForce::Mx] = map(requestPacket.ButtonL2, 0, 255, 0, 500) * (requestPacket.ButtonL1 == 0 ? 1 : -1);
    motorForces[MotorForce::My] = map(requestPacket.LeftStickY, 0, 255, -500, 500);
    motorForces[MotorForce::Mz] = map(requestPacket.RightStickX, 0, 255, -500, 500);

    //Расчет команд для двигателей
    int16_t motorCommands[6]{};
    for (uint8_t i = 0; i < 6; i++)
    {
      for (uint8_t j = 0; j < 6; j++)
      {
        motorCommands[i] += (int16_t) (motorForces[j] * (float)MotorCoefficients[i][j] / 100.0F);
      }
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
    const char replyBuffer = 'W'; // W = Working
    UDP.beginPacket(RemoteIP, Port);
    UDP.write(&replyBuffer);
    UDP.endPacket(); 
  }
  else
  {
    // Сброс флага
    MotorCoefficientsReceived = false;
  }
}
