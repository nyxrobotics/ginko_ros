int move_urg3_Motion_Loop[18][27]={
{  16,   25,    1,    2,    3,    5,    7,    8,    9,   10,   12,   14,   15,   16,   17,   19,   20,   21,   22,   24,   25,    4,    6,   11,   13,   18,   23},//(一個目モーションの総フレーム数、二個目サーボの総数、三個目以降が指定したID)
{   2,    0,    0, -130,   60,   60, -110,    0,  130,  -60,  -60, -110,  -40, -225,  625, 1300, 1300,  225, -625,-1300,-1300,   60,   60,  -60,  -60, -625,  625},//初期姿勢(一個目移動時間、二個目待機時間、三個目以降角度)
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  -57,  -11,  -11,    0,   57,   11,   11,    0,    0,    0,    0,   57,  -57},//以下モーションデータ(一個目移動時間、二個目待機時間、三個目以降角度)
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, -220,  -44,  -44,    0,  220,   44,   44,    0,    0,    0,    0,  220, -220},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, -463,  -93,  -93,    0,  463,   93,   93,    0,    0,    0,    0,  463, -463},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, -750, -150, -150,    0,  750,  150,  150,    0,    0,    0,    0,  750, -750},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,-1037, -207, -207,    0, 1037,  207,  207,    0,    0,    0,    0, 1037,-1037},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,-1280, -256, -256,    0, 1280,  256,  256,    0,    0,    0,    0, 1280,-1280},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,-1443, -289, -289,    0, 1443,  289,  289,    0,    0,    0,    0, 1443,-1443},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,-1500, -300, -300,    0, 1500,  300,  300,    0,    0,    0,    0, 1500,-1500},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,-1443, -289, -289,    0, 1443,  289,  289,    0,    0,    0,    0, 1443,-1443},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,-1280, -256, -256,    0, 1280,  256,  256,    0,    0,    0,    0, 1280,-1280},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,-1037, -207, -207,    0, 1037,  207,  207,    0,    0,    0,    0, 1037,-1037},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, -750, -150, -150,    0,  750,  150,  150,    0,    0,    0,    0,  750, -750},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, -463,  -93,  -93,    0,  463,   93,   93,    0,    0,    0,    0,  463, -463},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, -220,  -44,  -44,    0,  220,   44,   44,    0,    0,    0,    0,  220, -220},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  -57,  -11,  -11,    0,   57,   11,   11,    0,    0,    0,    0,   57,  -57},
{ 320,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}
};

