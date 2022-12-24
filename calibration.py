from imucal import ferraris_regions_from_section_list
from imucal import ferraris_regions_from_interactive_plot
from imucal import FerrarisCalibration
from imucal.management import save_calibration_info
from datetime import datetime
from pathlib import Path
import pandas as pd

data = pd.read_csv("C:\\Users\\Artem\\Desktop\\results.csv", header = 0, index_col=0)
data.head()

regions, section_list = ferraris_regions_from_interactive_plot(data)

cal = FerrarisCalibration()
cal_info = cal.compute(regions, sampling_rate_hz = 45.0, from_acc_unit="m/s^2", from_gyr_unit="deg/s", comment="my comment")

print(cal_info.to_json())

file_path = save_calibration_info(
    cal_info, sensor_id="imul", cal_time=datetime(2022, 12, 4), folder=("C:\\Users\\Artem\\Desktop\\")
)
file_path