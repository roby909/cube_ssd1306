#include <SPI.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define JOY_X A0
#define JOY_Y A1
#define JOY_SW 2

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUM_OF_INDICES 12

// Feel free to change these rotation speeds
float rotationXspeed = 0.09;
float rotationYspeed = 0.16;
float rotationZspeed = 0.3;

float cubeX = 0;
float cubeY = 0;
float cubeZ = 0;

int cubeOffset_x = SCREEN_WIDTH / 2;
int cubeOffset_y = SCREEN_HEIGHT / 2;
float cubeSize = 25;
float cameraDistance = 1;
float rotationX = 0;
float rotationY = 0;
float rotationZ = 0;

float vertices[][3] = { {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                      {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1} };

int indices[][2] = { {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7} };

void setup() {  
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  pinMode(JOY_SW, INPUT);
  digitalWrite(JOY_SW, HIGH);

  // Clear the buffer
  display.clearDisplay();
  display.display();
}

float* rotate(float& x, float& y, float angle)
{
  float s = sin(angle);
  float c = cos(angle);
  float* result = new float[2];
  result[0] = x * c - y * s;
  result[1] = y * c + x * s;
  return result;
}

void loop() {
  float xMovement, yMovement, zMovement;      
  xMovement = -(analogRead(JOY_X) - 512);
  if (xMovement > -10 && xMovement < 10)
    xMovement = 0;
  yMovement = -(analogRead(JOY_Y) - 512);
  if (yMovement > -10 && yMovement < 10)
    yMovement = 0;
  zMovement = digitalRead(JOY_SW);  

  xMovement /= 64;
  yMovement /= 64;
  zMovement -= 1;
  zMovement *= 0.05;

  cubeX += xMovement;
  cubeY += yMovement;
  cubeZ += zMovement;
    
  for (int i = 0; i < NUM_OF_INDICES; i++)
  {
    int* index = indices[i];
    int a = index[0];
    int b = index[1];

    float* vertex1 = vertices[a];
    float* vertex2 = vertices[b];

    float x1, y1, z1, x2, y2, z2;
    x1 = vertex1[0];
    y1 = vertex1[1];        
    z1 = vertex1[2];

    float* temp_Rotation = rotate(x1, z1, rotationY);
    x1 = temp_Rotation[0];
    z1 = temp_Rotation[1];
    delete temp_Rotation;

    temp_Rotation = rotate(y1, z1, rotationX);
    y1 = temp_Rotation[0];
    z1 = temp_Rotation[1];
    delete temp_Rotation;

    x1 = (float)(x1 * cubeSize);
    y1 = (float)(y1 * cubeSize);
    
    x2 = (float)vertex2[0];
    y2 = (float)vertex2[1];
    z2 = (float)vertex2[2];

    temp_Rotation = rotate(x2, z2, rotationY);
    x2 = temp_Rotation[0];
    z2 = temp_Rotation[1];
    delete temp_Rotation;

    temp_Rotation = rotate(y2, z2, rotationX);
    y2 = temp_Rotation[0];
    z2 = temp_Rotation[1];
    delete temp_Rotation;    
    
    x2 = (float)(x2 * cubeSize);
    y2 = (float)(y2 * cubeSize);            

    x1 += cubeX;
    x2 += cubeX;
    y1 += cubeY;
    y2 += cubeY;
    z1 += cubeZ;
    z2 += cubeZ;

    x1 /= z1 * cameraDistance;
    x2 /= z2 * cameraDistance;
    y1 /= z1 * cameraDistance;
    y2 /= z2 * cameraDistance;

    x1 += cubeOffset_x;
    x2 += cubeOffset_x;
    y1 += cubeOffset_y;
    y2 += cubeOffset_y;
    
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }

  rotationX += rotationXspeed;
  rotationY += rotationYspeed;
  rotationZ += rotationZspeed;
    
  display.display();
  display.clearDisplay();
}
