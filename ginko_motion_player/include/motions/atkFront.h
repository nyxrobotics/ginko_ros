int atkFront_Motion_Start[7][27]={
{   5,   25,    1,    2,    3,    5,    7,    8,    9,   10,   12,   14,   15,   16,   17,   19,   20,   21,   22,   24,   25,    4,    6,   11,   13,   18,   23},//(一個目モーションの総フレーム数、二個目サーボの総数、三個目以降が指定したID)
{   2,    0,    0, -130,   60,   60, -110,    0,  130,  -60,  -60, -110,  -40, -225,  625, 1300, 1300,  225, -625,-1300,-1300,   60,   60,  -60,  -60, -625,  625},//初期姿勢(一個目移動時間、二個目待機時間、三個目以降角度)
{  10,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},//以下モーションデータ(一個目移動時間、二個目待機時間、三個目以降角度)
{ 200,    0,    0,   50,    0,    0,   50,    0,  -50,    0,    0,   50, -150,  300, -150, -900, -900,    0,    0,    0,    0,    0,    0,    0,    0,  150,    0},
{ 200,    0,    0,   50,    0,    0,   50,    0,  -50,    0,    0,   50, -450,    0, -150,-1800,-1800,    0,    0,    0,    0,    0,    0,    0,    0,  150,    0},
{  10,  490,    0,   50, -100,  -50,   50,    0,  -50,  100,   50,   50, -450, -900, -450, -900, -900,    0,    0,    0,    0, -100,  -50,  100,   50,  450,    0},
{ 750,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}
};
