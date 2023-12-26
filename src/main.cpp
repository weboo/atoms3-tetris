//========================================================================
// TETRIS for M5ATOMS3
// 2023.05.11 Ported by weboo https://github.com/weboo
//
// Original code from
// https://github.com/m5stack/M5Stack/tree/master/examples/Games/Tetris
//
// Controller : Lean = LEFT/RIGHT, Shake = START, ROTATE
// Display    : Center = 110x120
// Block      : 8ea, 10x10 pixel
//========================================================================

#include <Arduino.h>
#include <M5Unified.h>

#define GREY 0x5AEB

uint16_t BlockImage[8][10][10];   // Block
uint16_t backBuffer[110][120];    // GAME AREA
const int Length = 10;            // the number of pixels for a side of a block
const int Width = 12;             // the number of horizontal blocks
const int Height = 11;            // the number of vertical blocks
int screen[Width][Height] = {0};  // it shows color-numbers of all positions
struct Point {
  int X, Y;
};
struct Block {
  Point square[4][4];
  int numRotate, color;
};
Point pos;
Block block;
int rot, fall_cnt = 0;
bool started = false, gameover = false;
boolean but_A = false, but_LEFT = false, but_RIGHT = false;
int game_speed = 500;  // 500msec
Block blocks[7] = {{{{{-1, 0}, {0, 0}, {1, 0}, {2, 0}},
                     {{0, -1}, {0, 0}, {0, 1}, {0, 2}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}}},
                    2,
                    1},
                   {{{{0, -1}, {1, -1}, {0, 0}, {1, 0}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}}},
                    1,
                    2},
                   {{{{-1, -1}, {-1, 0}, {0, 0}, {1, 0}},
                     {{-1, 1}, {0, 1}, {0, 0}, {0, -1}},
                     {{-1, 0}, {0, 0}, {1, 0}, {1, 1}},
                     {{1, -1}, {0, -1}, {0, 0}, {0, 1}}},
                    4,
                    3},
                   {{{{-1, 0}, {0, 0}, {0, 1}, {1, 1}},
                     {{0, -1}, {0, 0}, {-1, 0}, {-1, 1}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}}},
                    2,
                    4},
                   {{{{-1, 0}, {0, 0}, {1, 0}, {1, -1}},
                     {{-1, -1}, {0, -1}, {0, 0}, {0, 1}},
                     {{-1, 1}, {-1, 0}, {0, 0}, {1, 0}},
                     {{0, -1}, {0, 0}, {0, 1}, {1, 1}}},
                    4,
                    5},
                   {{{{-1, 1}, {0, 1}, {0, 0}, {1, 0}},
                     {{0, -1}, {0, 0}, {1, 0}, {1, 1}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
                     {{0, 0}, {0, 0}, {0, 0}, {0, 0}}},
                    2,
                    6},
                   {{{{-1, 0}, {0, 0}, {1, 0}, {0, -1}},
                     {{0, -1}, {0, 0}, {0, 1}, {-1, 0}},
                     {{-1, 0}, {0, 0}, {1, 0}, {0, 1}},
                     {{0, -1}, {0, 0}, {0, 1}, {1, 0}}},
                    4,
                    7}};

uint16_t score = 0;
uint8_t level = 1;

float ax, ay, az;
float pitch, roll;
const float threshold_shake = 1.5;
uint64_t rot_prevtime = 0;

void Draw() {
  // Draw 110x120 in the center
  for (int i = 0; i < Width; ++i)
    for (int j = 0; j < Height; ++j)
      for (int k = 0; k < Length; ++k)
        for (int l = 0; l < Length; ++l)
          backBuffer[j * Length + l][i * Length + k] =
              BlockImage[screen[i][j]][k][l];
  M5.Lcd.pushImage(4, 15, 120, 110, (uint16_t *)backBuffer);
}

void PutStartPos() {
  game_speed = 20;
  pos.X = 4;
  pos.Y = 1;
  block = blocks[random(7)];
  rot = random(block.numRotate);
}

bool GetSquares(Block block, Point pos, int rot, Point *squares) {
  bool overlap = false;
  for (int i = 0; i < 4; ++i) {
    Point p;
    p.X = pos.X + block.square[rot][i].X;
    p.Y = pos.Y + block.square[rot][i].Y;
    overlap |= p.X < 0 || p.X >= Width || p.Y < 0 || p.Y >= Height ||
               screen[p.X][p.Y] != 0;
    squares[i] = p;
  }
  return !overlap;
}

void GameOver() {
  for (int i = 0; i < Width; ++i)
    for (int j = 0; j < Height; ++j)
      if (screen[i][j] != 0) screen[i][j] = 4;
  gameover = true;
}

void ClearKeys() {
  but_A = false;
  but_LEFT = false;
  but_RIGHT = false;
}

bool detectShake() {
  if (millis() - rot_prevtime < 400) {
    return false;
  }

  M5.Imu.getAccel(&ax, &ay, &az);
  float a = sqrtf(pow(ax, 2.0) + pow(ay, 2.0) + pow(az, 2.0));
  if (a > threshold_shake) {
    rot_prevtime = millis();
    return true;
  }

  return false;
}

void calcRollPitch(float &roll, float &pitch) {
  float gx, gy, gz;
  M5.Imu.getGyro(&gx, &gy, &gz);

  roll = atan2(ay, az) * 180.0 / M_PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  float dt = 0.01;
  static float roll_prev = 0, pitch_prev = 0;
  roll += (gx / 131.0) * dt;
  pitch += (gy / 131.0) * dt;

  roll = roll * 0.98 + roll_prev * 0.02;
  pitch = pitch * 0.98 + pitch_prev * 0.02;
  roll_prev = roll;
  pitch_prev = pitch;
}

bool KeyPadLoop() {
  if (detectShake()) {
    ClearKeys();
    but_A = true;
    return true;
  }

  calcRollPitch(roll, pitch);

  int targetX;
  if (pitch < 0) {
    targetX = (int)(8 - (pitch * -0.1));
  } else {
    targetX = (int)(5 + (pitch * 0.1));
  }

  if (targetX > pos.X) {
    ClearKeys();
    but_RIGHT = true;
    return true;
  } else if (targetX < pos.X) {
    ClearKeys();
    but_LEFT = true;
    return true;
  }

  return false;
}

void GetNextPosRot(Point *pnext_pos, int *pnext_rot) {
  bool received = KeyPadLoop();

  if (but_LEFT || but_RIGHT) started = true;
  if (!started) return;

  pnext_pos->X = pos.X;
  pnext_pos->Y = pos.Y;
  if ((fall_cnt = (fall_cnt + 1) % 10) == 0)
    pnext_pos->Y += 1;
  else if (1) {
    if (but_LEFT) {
      but_LEFT = false;
      pnext_pos->X -= 1;
    } else if (but_RIGHT) {
      but_RIGHT = false;
      pnext_pos->X += 1;
    } else if (but_A) {
      but_A = false;
      *pnext_rot = (*pnext_rot + block.numRotate - 1) % block.numRotate;
    }
  }
}

void DeleteLine() {
  for (int j = 0; j < Height; ++j) {
    bool Delete = true;
    for (int i = 0; i < Width; ++i)
      if (screen[i][j] == 0) Delete = false;

    if (!Delete) continue;

    score++;
    if (score % 5 == 0) {
      level++;
      game_speed = game_speed - 50;
      if (game_speed < 100) game_speed = 100;
      M5.Lcd.drawString("LV:" + String(level), 88, 0, &fonts::Font0);
    }
    M5.Lcd.drawString("SCORE:" + String(score), 14, 0, &fonts::Font0);

    for (int k = j; k >= 1; --k) {
      for (int i = 0; i < Width; ++i) {
        screen[i][k] = screen[i][k - 1];
      }
    }
  }
}

void ReviseScreen(Point next_pos, int next_rot) {
  if (!started) return;

  Point next_squares[4];
  for (int i = 0; i < 4; ++i)
    screen[pos.X + block.square[rot][i].X][pos.Y + block.square[rot][i].Y] = 0;
  if (GetSquares(block, next_pos, next_rot, next_squares)) {
    for (int i = 0; i < 4; ++i) {
      screen[next_squares[i].X][next_squares[i].Y] = block.color;
    }
    pos = next_pos;
    rot = next_rot;
  } else {
    for (int i = 0; i < 4; ++i)
      screen[pos.X + block.square[rot][i].X][pos.Y + block.square[rot][i].Y] =
          block.color;
    if (next_pos.Y == pos.Y + 1) {
      DeleteLine();
      PutStartPos();
      if (!GetSquares(block, pos, rot, next_squares)) {
        for (int i = 0; i < 4; ++i)
          screen[pos.X + block.square[rot][i].X]
                [pos.Y + block.square[rot][i].Y] = block.color;
        GameOver();
      }
    }
  }
  Draw();
}

void restart() {
  for (int j = 0; j < Height; ++j) {
    for (int i = 0; i < Width; ++i) {
      screen[i][j] = 0;
    }
  }

  gameover = false;
  score = 0;
  game_speed = 20;
  level = 1;
  PutStartPos();

  for (int i = 0; i < 4; ++i) {
    screen[pos.X + block.square[rot][i].X][pos.Y + block.square[rot][i].Y] =
        block.color;
  }
  M5.Lcd.drawString("SCORE:" + String(score), 14, 0, &fonts::Font0);
  M5.Lcd.drawString("LV:" + String(level), 88, 0, &fonts::Font0);
  Draw();
}

void make_block(int n, uint16_t color) {
  // Make Block color
  for (int i = 0; i < Length; i++)
    for (int j = 0; j < Length; j++) {
      BlockImage[n][i][j] = color;                    // Block color
      if (i == 0 || j == 0) BlockImage[n][i][j] = 0;  // BLACK Line
    }
}

void setup(void) {
  auto cfg = M5.config();
  cfg.internal_imu = true;
  M5.begin(cfg);
  M5.Imu.begin();
  M5.Lcd.init();

  M5.Lcd.setSwapBytes(true);
  M5.Lcd.fillScreen(BLACK);

  M5.Lcd.drawLine(3, 14, 124, 14, GREY);
  M5.Lcd.drawLine(3, 125, 124, 125, GREY);
  M5.Lcd.drawLine(3, 14, 3, 125, GREY);
  M5.Lcd.drawLine(124, 14, 124, 125, GREY);

  M5.Lcd.drawString("SCORE:" + String(score), 14, 0, &fonts::Font0);
  M5.Lcd.drawString("LV:" + String(level), 88, 0, &fonts::Font0);

  //----------------------------// Make Block ----------------------------
  make_block(0, BLACK);   // Type No, Color
  make_block(1, 0x00F0);  // DDDD     RED
  make_block(2, 0xFBE4);  // DD,DD    PUPLE
  make_block(3, 0xFF00);  // D__,DDD  BLUE
  make_block(4, 0xFF87);  // DD_,_DD  GREEN
  make_block(5, 0x87FF);  // __D,DDD  YELLO
  make_block(6, 0xF00F);  // _DD,DD_  LIGHT GREEN
  make_block(7, 0xF8FC);  // _D_,DDD  PINK
  //----------------------------------------------------------------------

  PutStartPos();
  for (int i = 0; i < 4; ++i) {
    screen[pos.X + block.square[rot][i].X][pos.Y + block.square[rot][i].Y] =
        block.color;
  }
  Draw();
}

void loop() {
  if (gameover) {
    if (detectShake()) restart();
  } else {
    Point next_pos;
    int next_rot = rot;
    GetNextPosRot(&next_pos, &next_rot);
    ReviseScreen(next_pos, next_rot);
  }

  // SPEED ADJUST
  delay(game_speed);
  M5.update();
}
