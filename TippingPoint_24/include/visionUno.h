/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature YELLOWMG = vex::vision::signature (1, 1275, 2237, 1756, -3277, -2467, -2872, 2.9, 0);
vex::vision::signature GREENFLAG = vex::vision::signature (2, -2381, -1837, -2109, -2759, -2161, -2460, 3, 0);
vex::vision::signature REDMG = vex::vision::signature (3, 4655, 9339, 6997, -729, 121, -304, 2.1, 0);
vex::vision::signature BLUEMG = vex::vision::signature (4, -2621, -1781, -2201, 7081, 12577, 9829, 1.4, 0);
vex::vision::signature BLAC = vex::vision::signature (5, -51, 51, 0, -51, 51, 0, 2.3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision VisionSenso = vex::vision (vex::PORT10, 50, YELLOWMG, GREENFLAG, REDMG, BLUEMG, BLAC, SIG_6, SIG_7);
/*vex-vision-config:end*/