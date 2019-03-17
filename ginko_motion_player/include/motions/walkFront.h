int walkFront_Motion_Start[3][27]={
{   1,   25,    1,    2,    3,    5,    7,    8,    9,   10,   12,   14,   15,   16,   17,   19,   20,   21,   22,   24,   25,    4,    6,   11,   13,   18,   23},//(一個目モーションの総フレーム数、二個目サーボの総数、三個目以降が指定したID)
{  20,    0,    0, -100,    0,    0, -100,    0,  100,    0,    0, -100,  -40, -225,  625, 1300, 1300,  225, -625,-1300,-1300,  -74,   22,   34,  -64, -685,  630},//初期姿勢(一個目移動時間、二個目待機時間、三個目以降角度)
{  33,    0,    0,  200,  100,  200,  100,    0,  125,    0, -250,  -85, -100,    0,    0,    0,    0,    0,    0,    0,    0,  100,  200,    0, -250,    0,    0}//以下モーションデータ(一個目移動時間、二個目待機時間、三個目以降角度)
};

int walkFront_Motion_Loop[10][27]={
{   8,   25,    1,    2,    3,    5,    7,    8,    9,   10,   12,   14,   15,   16,   17,   19,   20,   21,   22,   24,   25,    4,    6,   11,   13,   18,   23},//(一個目モーションの総フレーム数、二個目サーボの総数、三個目以降が指定したID)
{  20,    0,    0, -100,    0,    0, -100,    0,  100,    0,    0, -100,  -40, -225,  625, 1300, 1300,  225, -625,-1300,-1300,  -74,   22,   34,  -64, -685,  630},//初期姿勢(一個目移動時間、二個目待機時間、三個目以降角度)
{  33,    0,    0,  150,   50,  300,   50,    0,   30, -300,    0,  -50, -100,    0,    0,    0,    0,    0,    0,    0,    0,   50,  300, -300,    0,    0,    0},//以下モーションデータ(一個目移動時間、二個目待機時間、三個目以降角度)
{  88,    0,    0,  100,  100,  350,   50,    0,   30,   50,  450,    0, -200,    0,    0,    0,    0,    0,    0,    0,    0,  100,  350,   50,  450,    0,    0},
{  66,    0,    0,  100,  200,  350,  100,    0,   50,  100,    0,  -50, -150,    0,    0,    0,    0,    0,    0,    0,    0,  200,  350,  100,    0,    0,    0},
{  33,    0,    0,   60,  300,  500,  100,    0,  -20, -100,  -50,    0, -100,    0,    0,    0,    0,    0,    0,    0,    0,  300,  500, -100,  -50,    0,    0},
{  33,    0,    0,  -30,  300,    0,  -50,    0, -150,  -50, -300,   50,  100,    0,    0,    0,    0,    0,    0,    0,    0,  300,    0,  -50, -300,    0,    0},
{  88,    0,    0,  -30,  -50, -450,    0,    0, -100, -100, -350,   50,  200,    0,    0,    0,    0,    0,    0,    0,    0,  -50, -450, -100, -350,    0,    0},
{  66,    0,    0,  -50, -100,    0,  -50,    0, -100, -200, -350,  100,  150,    0,    0,    0,    0,    0,    0,    0,    0, -100,    0, -200, -350,    0,    0},
{  33,    0,    0,   20,  100,   50,    0,    0,  -60, -300, -500,  100,  100,    0,    0,    0,    0,    0,    0,    0,    0,  100,   50, -300, -500,    0,    0}
};

int walkFront_Motion_End[6][27]={
{   4,   25,    1,    2,    3,    5,    7,    8,    9,   10,   12,   14,   15,   16,   17,   19,   20,   21,   22,   24,   25,    4,    6,   11,   13,   18,   23},//(一個目モーションの総フレーム数、二個目サーボの総数、三個目以降が指定したID)
{  20,    0,    0, -100,    0,    0, -100,    0,  100,    0,    0, -100,  -40, -225,  625, 1300, 1300,  225, -625,-1300,-1300,  -74,   22,   34,  -64, -685,  630},//初期姿勢(一個目移動時間、二個目待機時間、三個目以降角度)
{  99,    0,    0,   70, -200,    0,   70,    0,   30, -250,  250, -150,   80,    0,    0,    0,    0,    0,    0,    0,    0, -200,    0, -250,  250,    0,    0},//以下モーションデータ(一個目移動時間、二個目待機時間、三個目以降角度)
{  66,    0,    0,   70, -200, -100,   70,    0,  -20,  450,  150,  -30, -450,    0,    0,    0,    0,    0,    0,    0,    0, -200, -100,  450,  150,    0,    0},
{  33,    0,    0,   70, -200, -150,   70,    0,  -20,  250,  150,  -30,    0,    0,    0,    0,    0,    0,    0,    0,    0, -200, -150,  250,  150,    0,    0},
{  66,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}
};

