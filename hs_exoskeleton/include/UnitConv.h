#ifndef UNITCONV_H
#define UNITCONV_H

#define RAD2DEG(x) ((x) * 57.2957795130823)
#define DEG2RAD(x) ((x) * 0.0174532925199433)

#define RAD2RPM(x) ((x) * 9.54929658551372)
#define RPM2RAD(x) ((x) * 0.10471975511966)

#define SELFMOTORCUR2CNT(x) ((x) * 100)

#define SELFMOTORPOSMODEVELLIMIT(x) ((x) * 10)

#define SELFMOTORPOSMODECURLIMIT(x) ((x) * 2.5)

#define SECOND2MILLISECOND(x_s) ((uint32_t)(x_s * 1000.0))

#endif // !UNITCONV_H
