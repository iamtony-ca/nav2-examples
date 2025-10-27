# 상위 상태(요구 테이블 그대로 매핑)
uint8 STATUS_INIT                         = 0
uint8 STATUS_PATH_SEARCHING               = 1
uint8 STATUS_WAITING_FOR_OBS              = 2
uint8 STATUS_MOVING                       = 3
uint8 STATUS_ARRIVED                      = 4
uint8 STATUS_MARKING                      = 5
uint8 STATUS_UNLOADING                    = 6
uint8 STATUS_UNLOADED                     = 7
uint8 STATUS_LOADING                      = 8
uint8 STATUS_LOADED                       = 9
uint8 STATUS_AUTORECOVERY                 = 10
uint8 STATUS_RECOVERING                   = 11
uint8 STATUS_MANUAL_RUNNING               = 12
uint8 STATUS_MANUAL_COMPLETE              = 13
uint8 STATUS_ERROR                        = 14
uint8 STATUS_PAUSE                        = 15
uint8 STATUS_WAITING_FOR_SAFETY           = 16
uint8 STATUS_WAITING_FOR_FLOWCONTROL      = 17
uint8 STATUS_WAITING_FOR_ROS_STATUS       = 18
uint8 STATUS_CHARGING                     = 19
uint8 STATUS_CHARGE_DONE                  = 20
uint8 STATUS_UNKNOWN                      = 21

uint8 phase