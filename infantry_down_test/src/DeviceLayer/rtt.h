#ifndef RTT_H
#define RTT_H
#include <SEGGER_RTT.h>

#define RTT_CH_LOG 0	// 通道 0 RTT 日志输出通道
#define RTT_CH_VOFA_1 1 // 通道 1
#define RTT_CH_VOFA_2 2 // 通道 2

#ifdef __cplusplus
extern "C" {
#endif

void vofa_rtt_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTT_H */