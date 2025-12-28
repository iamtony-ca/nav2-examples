from datetime import datetime
from zoneinfo import ZoneInfo
import time

# [수정됨] 마이크로초(.%f)를 포함한 포맷 정의
# %f는 6자리 마이크로초를 의미합니다 (예: .123456)
SEOUL_TZ = ZoneInfo("Asia/Seoul")
DEFAULT_FMT = '%Y-%m-%d %H:%M:%S.%f'

class TimeConverter:
    """
    Unix Timestamp와 Seoul Time(Asia/Seoul) 간의 양방향 변환 유틸리티
    (마이크로초 정밀도 지원)
    """

    @staticmethod
    def unix_to_seoul(unix_timestamp: float, fmt: str = DEFAULT_FMT) -> str:
        """
        Unix Timestamp(float) -> Seoul Time 문자열 변환
        """
        dt_seoul = datetime.fromtimestamp(unix_timestamp, SEOUL_TZ)
        return dt_seoul.strftime(fmt)

    @staticmethod
    def seoul_to_unix(time_str: str, fmt: str = DEFAULT_FMT) -> float:
        """
        Seoul Time 문자열 -> Unix Timestamp(float) 변환
        """
        # 1. 문자열 파싱 (Naive datetime)
        dt_naive = datetime.strptime(time_str, fmt)
        
        # 2. 타임존 정보 주입 (Aware datetime)
        dt_aware = dt_naive.replace(tzinfo=SEOUL_TZ)
        
        # 3. Unix Timestamp 반환
        return dt_aware.timestamp()

# --- Main 실행 예제 ---
if __name__ == "__main__":
    # 1. 현재 시간을 기준으로 테스트 (float 정밀도 유지)
    current_ts = time.time()
    
    # 출력 시 소수점 6자리까지 표현 (.6f)
    print(f"--- [1] Unix ({current_ts:.6f}) -> Seoul Time ---")
    seoul_str = TimeConverter.unix_to_seoul(current_ts)
    print(f"Result: {seoul_str} (KST)")

    print(f"\n--- [2] Seoul Time ('{seoul_str}') -> Unix ---")
    converted_ts = TimeConverter.seoul_to_unix(seoul_str)
    print(f"Result: {converted_ts:.6f}")

    # 검증
    diff = abs(current_ts - converted_ts)
    print(f"\n[검증] 오차(초): {diff:.9f}")
    
    if diff < 1e-6:
        print(">> 성공: 마이크로초 단위까지 정확하게 변환되었습니다.")
    else:
        print(">> 주의: 부동소수점 오차가 발생했습니다.")