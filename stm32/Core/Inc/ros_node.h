#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* main.c(C)에서 호출하는 진입점들 */
void ROS_Init(void);
void ROS_SpinOnce(void);

/* /state_msg, /lidar_distance 구독 결과를 main.c의 주행 판단 로직에서 사용 */
int32_t ROS_GetState(void);       /* 0=정지, 1=수동(정지 처리), 2=자동, -1=초기값 */
int      ROS_ObstacleNear(void);  /* 1이면 0.5m 이내 장애물 감지됨 */

/* 헬스체크/테스트용 퍼블리시 */
void ROS_PublishEcho(int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* ROS_NODE_H */
