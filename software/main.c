#include <Pixy2.h>
#include <tcs3200.h>

Pixy2 pixy;

#define deadZone 0.15  // 로봇이 전방에 있다고 인식하는 범위
#define RBS 20         // 기본 속도 (0 ~ 230)
#define RMS 200        // 최대 속도 (0 ~ 230)
#define DEBUG 0
#define CAMERA_DETEC 100 // 카메라가 놓쳤을 때 돌던 방향으로 얼마나 더 돌지 (클수록 많이 돔)
//------------전역변수--------------------------------------------------------------
int width, high;
int Size_[] = { 0, 0, 0, 0, 0 };
float power;
int red_left, green_left, blue_left; 
int red_right, green_right, blue_right;
//float cx; 
int recognize;
int left_color_re;
int right_color_re;
int cont = 0;
int x;
long dis;
unsigned long previousMillis = 0;    // 마지막 초음파 측정 시간 저장
const long interval = 200;           // 초음파 측정 간격 200ms
// 미리 정의된 색상들의 RGB 값 (빨간색, 파란색, 검정색, 노란색, 흰색) 값에 맞춰서 튜닝해야함
int redColor[3] = {20, 0, 0};
int blueColor[3] = {0, 300, 30};
int blackColor[3] = {0, 0, 0};
int yellowColor[3] = {30, 30, 0};

unsigned long lastDebounceTime_left = 0;
unsigned long lastDebounceTime_right = 0;
unsigned long debounceDelay = 50;  // 50ms 디바운스 지연 시간
String stableLeftColor = "Unknown";
String stableRightColor = "Unknown";


// 각 색상에 대응하는 이름들
String colorNames[7] = {"Red", "Blue", "Black_low","Black_high", "Yellow_low","Yellow_high"};
int colorRGB_right[7][3] = {{200,50,90}, {15,33,200}, {5,5,5},{20,30,35}, {47,50,30},{110,110,70}};
int colorRGB_left[7][3] = {{200,50,90}, {15,33,200}, {5,5,5}, {20,30,35},{47,50,30},{110,110,70}};

//------------핀번호--------------------------------------------------------------
// 왼쪽 위
const int left_top_pwm = 11;
const int left_top_dir = 10;
// 왼쪽 아래
const int left_down_pwm = 13;
const int left_down_dir = 12;
// 오른쪽 위
const int right_top_pwm = 6;
const int right_top_dir = 7;
// 오른쪽 아래
const int right_down_pwm = 4;
const int right_down_dir = 5;

//적외선 센서
int ir_right = 46;  // 오른쪽 적외선 센서가 연결된 핀
int ir_left = 44;  // 왼쪽 적외선 센서가 연결된 핀
tcs3200 tcs_left(41, 39, 37, 35, 33); // 왼쪽 컬러센서(S0, S1, S2, S3, output pin) 
tcs3200 tcs_right(40,38,36,34,32); // 오른쪽 컬러센서(S0, S1, S2, S3, output pin)
//----------------------------------함수 선언----------------------------------------------

float mapfloat(long , long , long , long , long); // 값 범위 변환 함수
float Robot_Power(); // 인식된 로봇의 크기에 따라 모터 출력 반환 + 급제동하여 범퍼 파고들기
String identifyColor(int , int , int); //색상 간의 유클리드 거리를 계산하여 가장 가까운 색상을 찾는 함수 
void base_motor(int , int ,int , int); //모터 설정 함수
void motor_control(int , int); //모터 컨트롤 함수 +: 정방향 -: 역방향 
void posMove(float, int);      //왼쪽, 가운데, 오른쪽으로 나눠서 모터가 움직이는 함수
void moveBackAndTurn(bool);   //뒤로 가고 턴하는 함수
void updateColorSensors();    //컬러센서 값 받아오는 함수
float Object_orientation();   // 좌, 우, 가운데 -1~1로 리턴하는 함수
bool isObstacleDetected(int );  //앞의 벽을 만나면 1을 return하는 함수
int printObstacleStatus(int );  //장애물만나면 1을 return하는 함수(시리얼 프린트 함수)

//-----------------------------------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial1.begin(9600); 
  Serial.print("Startin.../n");

  pinMode(ir_right, INPUT);  // 적외선 센서 핀을 입력 모드로 설정
  pinMode(ir_left, INPUT);  // 적외선 센서 핀을 입력 모드로 설정
  
  pinMode(left_top_pwm, OUTPUT);  // 왼쪽 위 모터 PWM 출력 설정
  pinMode(left_top_dir, OUTPUT);  // 왼쪽 위 모터 방향 출력 설정
  pinMode(left_down_pwm, OUTPUT); // 왼쪽 아래 모터 PWM 출력 설정
  pinMode(left_down_dir, OUTPUT); // 왼쪽 아래 모터 방향 출력 설정
  pinMode(right_top_pwm, OUTPUT); // 오른쪽 위 모터 PWM 출력 설정
  pinMode(right_top_dir, OUTPUT); // 오른쪽 위 모터 방향 출력 설정
  pinMode(right_down_pwm, OUTPUT);// 오른쪽 아래 모터 PWM 출력 설정
  pinMode(right_down_dir, OUTPUT);// 오른쪽 아래 모터 방향 출력 설정
  
  digitalWrite(left_top_pwm, LOW);
  digitalWrite(left_top_dir, LOW);
  digitalWrite(left_down_pwm, LOW);
  digitalWrite(left_down_dir, LOW);
  digitalWrite(right_top_pwm, LOW);
  digitalWrite(right_top_dir, LOW);
  digitalWrite(right_down_pwm, LOW);
  digitalWrite(right_down_dir, LOW);

  pixy.init();
}
//--------------------------------------------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  // Pixy 카메라 처리
  recognize = pixy.ccc.getBlocks();  // Pixy2로 물체 인식
  float turn = Object_orientation();
  updateColorSensors(); // 색상 센서 업데이트

  recognize = pixy.ccc.getBlocks(); //인식하면 1

  if (left_color_re == 0 && right_color_re == 0) 
  { // 검정이나 노랑
    
    if(recognize == 1) // 로봇이 인식된 경우
    {
      
      int p = Robot_Power();
      //Serial.println(turn);
      posMove(turn,p);
    }
    else // 로봇이 인식되지 않은 경우
    {
      
      if(isObstacleDetected(ir_left)) //왼쪽 벽 인식
      {
        motor_control(-RBS * 4, -RBS * 2); // 오른쪽 바퀴가 뒤로 더 감
        printObstacleStatus(ir_left);
        delay(300);
      }
      else if(isObstacleDetected(ir_right)) //오른쪽 벽 인식
      {
        motor_control(-RBS * 2, -RBS * 4); // 왼쪽 바퀴가 뒤로 더 감
        printObstacleStatus(ir_right);
        delay(300);
      }
      else
      {
        motor_control(-RBS * 3, RBS * 3);
      }
    }
  }
  else if (left_color_re == 1 && right_color_re == 0) //왼쪽 빨강이나 파랑
  {
    moveBackAndTurn(true);
  }
  else //오른쪽 빨강이나 파랑
  {
    moveBackAndTurn(false);
  }
}

//값 범위를 변환시켜줌
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// 후진 및 회전 함수
void moveBackAndTurn(bool isLeft) {
  //for (int i = 0; i < 3000; i++) 
  motor_control(-RBS * 2, -RBS * 2); // 후진
  delay(200);
  unsigned long startTime = 0; // 색상이 감지된 시작 시간을 저장
  bool colorDetected = false; // 색상 감지 상태
  while (true) {
    updateColorSensors();
    if (isLeft) {
      //updateColorSensors();
      motor_control(-RBS * 4, -RBS * 2); // 왼쪽 회전
      Serial.println("left_color");
      if (left_color_re == 1) {
        if (!colorDetected) {
          // 색상이 처음 감지된 순간 시간 기록
          startTime = millis();
          colorDetected = true;
        } else if (millis() - startTime >= 500) {
          // 1초 이상 색상이 감지되었을 때
          Serial.println("1초 이상 왼쪽 색상이 감지됨");
          while(1)
          {
            int p = Robot_Power();
            float turn = Object_orientation();
            posMove(turn,p); 
            updateColorSensors();
            if(left_color_re == 0) break;
          }
          // 여기서 원하는 로직을 실행
          break;
        }
      } else {
        // 색상이 감지되지 않으면 초기화
        colorDetected = false;
      }
      if (left_color_re == 0) break;
    } else {
      //updateColorSensors();
      motor_control(-RBS * 2, -RBS * 4); // 오른쪽 회전
      Serial.println("right_color");
      if (right_color_re == 1) {
        if (!colorDetected) {
          // 색상이 처음 감지된 순간 시간 기록
          startTime = millis();
          colorDetected = true;
        } else if (millis() - startTime >= 500) {
          // 1초 이상 색상이 감지되었을 때
          Serial.println("1초 이상 오른쪽 색상이 감지됨");
          // 여기서 원하는 로직을 실행
          while(1)
          {
            int p = Robot_Power();
            float turn = Object_orientation();
            posMove(turn,p); 
            updateColorSensors();
            if(right_color_re == 0) break;
          }          
          break;
        }
      } else {
        // 색상이 감지되지 않으면 초기화
        colorDetected = false;
      }
      if (right_color_re == 0) break;
    }
  }
}

float Object_orientation()  // 물체의 방향 반환
{
  uint16_t blocks;
  float cx;
  blocks = pixy.ccc.getBlocks();

  if (blocks)  // 인식된 경우
  {
    x = pixy.ccc.blocks[0].m_x;  // 0번째 인식된 물체의 x좌표 반환
    cx = x;
    cx = mapfloat(cx, 3, 310, -1, 1);  // x좌표를 -1에서 1사이의 값으로 변환
  } else {                             // 인식되지 않은 경우 이전에 저장된 물체의 위치를 반환
    cont += 1;                         // 인식시도 횟수 증가
    if (cont == CAMERA_DETEC) {        // 인식시도 횟수가 정해진 숫자가 되면
      cont = 0;                        // 인식시도 횟수 초기화
      cx = 0;                          // 방향 정면으로 설정
    }
  }
  return cx;
}

// 상대로봇의 인식된크기 (거리) 에 따른 모터출력 반환
float Robot_Power() {

  width = pixy.ccc.blocks[0].m_width;  // 인식된 로봇의 너비
  high = pixy.ccc.blocks[0].m_height;  // 인식된 로봇의 높이
  float power_ = 0;
  float Size = width * high;  // 넓이 계산
  float Size_sum= 0 ;
  // 인식 대상의 크기를 필터링
  for (int i = 0; i < 4; i++) {  // 저장되어 있던 넓이 값을 한칸씩 이동
    Size_[i] = Size_[i + 1];
  }
  Size_[4] = Size;               // 배열의 마지막에 새로 계산된 넓이 업데이트
  for (int i = 0; i < 5; i++) {  // 배열의 모든값을 더함
    Size_sum = Size_sum + Size_[i];
  }
  // 최솟값을 RBS 최댓값을 RMS가 되도록 선형출력 계산
  if (Size < 3000) {
    power_ = Size_sum / 5 * (RMS - RBS) / 7000 + RBS;
  } else if(Size > 3000 & Size < 3400){
    power_ = -RMS;  // 범퍼 파고들기
  } else{
    power_ = RMS;  // 최대속도
  }

  //Serial.println("size: " + String(Size) + ", power: " + String(power_));
  return power_;
}

//색 감지용 함수
// 왼쪽 색상 감지 함수
String identifyLeftColor(int r, int g, int b) {
  int closestIndex = 0;
  int minDistance = 255 * 3;

  for (int i = 0; i < 6; i++) {
    int distance = sqrt(pow(colorRGB_left[i][0] - r, 2) + pow(colorRGB_left[i][1] - g, 2) + pow(colorRGB_left[i][2] - b, 2));
    if (distance < minDistance) {
      minDistance = distance;
      closestIndex = i;
    }
  }
  return colorNames[closestIndex];
}

// 오른쪽 색상 감지 함수
String identifyRightColor(int r, int g, int b) {
  int closestIndex = 0;
  int minDistance = 255 * 3;

  for (int i = 0; i < 6; i++) {
    int distance = sqrt(pow(colorRGB_right[i][0] - r, 2) + pow(colorRGB_right[i][1] - g, 2) + pow(colorRGB_right[i][2] - b, 2));
    if (distance < minDistance) {
      minDistance = distance;
      closestIndex = i;
    }
  }
  return colorNames[closestIndex];
}

// 색상 감지 및 업데이트 함수 수정
void updateColorSensors() {
  red_left = tcs_left.colorRead('r',100);
  green_left = tcs_left.colorRead('g',100);
  blue_left = tcs_left.colorRead('b',100);

  red_right = tcs_right.colorRead('r',100);
  green_right = tcs_right.colorRead('g',100);
  blue_right = tcs_right.colorRead('b',100);

  // 왼쪽 색상 감지
  String leftColor = identifyLeftColor(red_left, green_left, blue_left);
  //Serial.println(leftColor);
  left_color_re = (leftColor == "Red" || leftColor == "Blue") ? 1 : 0;

  // 오른쪽 색상 감지
  String rightColor = identifyRightColor(red_right, green_right, blue_right);
  //Serial.println(rightColor);
  right_color_re = (rightColor == "Red" || rightColor == "Blue") ? 1 : 0;
}
// void updateColorSensors() {
//   red_left = tcs_left.colorRead('r', 100);
//   green_left = tcs_left.colorRead('g', 100);
//   blue_left = tcs_left.colorRead('b', 100);

//   red_right = tcs_right.colorRead('r', 100);
//   green_right = tcs_right.colorRead('g', 100);
//   blue_right = tcs_right.colorRead('b', 100);

//   // 왼쪽 색상 감지
//   String currentLeftColor = identifyLeftColor(red_left, green_left, blue_left);
//   if (currentLeftColor != stableLeftColor) {
//     lastDebounceTime_left = millis();  // 색상 값이 변하면 시간을 갱신
//   }
//   if ((millis() - lastDebounceTime_left) > debounceDelay) {
//     stableLeftColor = currentLeftColor;  // 일정 시간 동안 유지되면 색상을 업데이트
//   }
//   left_color_re = (stableLeftColor == "Red" || stableLeftColor == "Blue") ? 1 : 0;

//   // 오른쪽 색상 감지
//   String currentRightColor = identifyRightColor(red_right, green_right, blue_right);
//   if (currentRightColor != stableRightColor) {
//     lastDebounceTime_right = millis();  // 색상 값이 변하면 시간을 갱신
//   }
//   if ((millis() - lastDebounceTime_right) > debounceDelay) {
//     stableRightColor = currentRightColor;  // 일정 시간 동안 유지되면 색상을 업데이트
//   }
//   right_color_re = (stableRightColor == "Red" || stableRightColor == "Blue") ? 1 : 0;
// }
// 모터 값 설정
void base_motor(int pin_dir, int pin_pwm,int dir, int speed)
{
  digitalWrite(pin_dir,dir);
  analogWrite(pin_pwm, speed);
}


// 모터 컨트롤
void motor_control(int R_motor_power, int L_motor_power){
  if(R_motor_power>=0){
    base_motor(left_top_dir,left_top_pwm,1,abs(R_motor_power));
    base_motor(left_down_dir,left_down_pwm,1,abs(R_motor_power));
  }
  else{
    base_motor(left_top_dir,left_top_pwm,0,abs(R_motor_power));
    base_motor(left_down_dir,left_down_pwm,0,abs(R_motor_power));
  }
  if(L_motor_power>=0){
    base_motor(right_top_dir,right_top_pwm,1,abs(L_motor_power));
    base_motor(right_down_dir,right_down_pwm,1,abs(L_motor_power));
  }
  else{
    base_motor(right_top_dir,right_top_pwm,0,abs(L_motor_power));
    base_motor(right_down_dir,right_down_pwm,0,abs(L_motor_power));
  }
}

void posMove(float cx,int p)
{
  if(cx<-deadZone) // 로봇의 방향이 왼쪽인 경우
    {
      motor_control(RBS,RBS*5);
    }
    else if((-deadZone<cx) && (cx<deadZone)) // 로봇의 방향이 오른쪽인 경우
    {
      motor_control(p,p);
    }
    else //로봇의 방향이 정면인 경우
    {
      motor_control(RBS*5,RBS);
    }
}

// 적외선 센서 값을 읽는 함수
bool isObstacleDetected(int pin) {
  int sensorValue = digitalRead(pin);  // 센서 값 읽기
  return (sensorValue == LOW);  // LOW 값이면 장애물 감지됨
}

// 장애물 감지 여부를 시리얼로 출력하는 함수
int printObstacleStatus(int pin) {
  if (isObstacleDetected(pin)) {
    Serial.println("장애물 감지됨!");
  } else {
    Serial.println("장애물 없음.");
  }
  return (isObstacleDetected(pin));
}
