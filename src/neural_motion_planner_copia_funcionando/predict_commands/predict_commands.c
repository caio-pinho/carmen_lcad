#include "seqdense.h"
#include "eml_net.h"
#include "eml_common.h"
#include <math.h>

int main(int argc, char* argv[])
{
float values[9];//= { 7757527.502,-363658.677,-0.154,8.550,7757499.058,-363648.787,-0.527,7.656,0.033};
//7757527.502 -363658.677 -0.154 8.550 7757499.058 -363648.787 -0.527 7.656 0.033
float out[3];

for (int i = 0; i < argc-1; i++) {
    values[i] = atof(argv[i+1]);
}

int err = seqdense_regress(values, 9, out, 3);
//EmlError err = nnmodel_regress(values, 6, out, 2);
printf("%f\n",out[0]);
printf("%f\n",out[1]);
printf("%f\n",out[2]);
if (err != EmlOk)
{
    printf("error");
}
else {
    printf("sem erros");
}

}