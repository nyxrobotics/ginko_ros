int attackRightBack2_Motion_Start[7][27]={
{   5,   25,    1,    2,    3,    5,    7,    8,    9,   10,   12,   14,   15,   16,   17,   19,   20,   21,   22,   24,   25,    4,    6,   11,   13,   18,   23},//(一個目モーションの総フレーム数、二個目サーボの総数、三個目以降が指定したID)
{  20,    0,    0, -100,    0,    0, -100,    0,  100,    0,    0, -100,  -40, -225,  625, 1300, 1300,  225, -625,-1300,-1300,  -74,   22,   34,  -64, -685,  630},//初期姿勢(一個目移動時間、二個目待機時間、三個目以降角度)
{  10,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},//以下モーションデータ(一個目移動時間、二個目待機時間、三個目以降角度)
{ 200,    0,    0,   50,    0,    0,   50,    0,  -50,    0,    0,   50,  150,  300,    0, -900, -900,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
{ 300,    0,    0,   50,    0,    0,   50,    0,  -50,    0,    0,   50,  450,  600, -300,-1200,-1200,    0,    0,    0,    0,    0,    0,    0,    0,  300,    0},
{ 100,  400,    0,   50,   50,  100,   50,    0,  -50,  -50, -100,   50,  450,  900, -450,-2500,-2500,    0,    0,    0,    0,   50,  100,  -50, -100,  450,    0},
{ 750,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}
};

