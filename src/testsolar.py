# import datetime
import datetime
from pytz import timezone
from pysolar.solar import *

# Định nghĩa vĩ độ, kinh độ và thời gian quan sát
latitude = 10.950621  # Vĩ độ
longitude = 106.555974  # Kinh độ
# 6/20/2024 21:42:01
date= dt = datetime.datetime(2024, 6, 21, 13, 30, 1, tzinfo=timezone('Asia/Ho_Chi_Minh'))

# Tính toán độ cao của Mặt Trời
altitude = get_altitude(latitude, longitude, date)
print(f"Altitude (degrees): {altitude:.2f}")
azimuth = get_azimuth(latitude,longitude,date)
print(f"Azimuth (degrees): {azimuth:.2f}")
