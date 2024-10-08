# functions/process_drone_files.py

import pandas as pd
import numpy as np
from scipy.interpolate import CubicSpline, Akima1DInterpolator
import os


def process_drone_files(skybrush_dir, processed_dir, method='cubic', dt=0.05):
    """
    Function to process drone files from a specified directory and output to another directory.

    Args:
    skybrush_dir (str): Thư mục chứa các tập tin drone sẽ được xử lý.
    processed_dir (str): Thư mục nơi các tập tin được xử lý sẽ được xuất ra
    method (str): Phương pháp nội suy được sử dụng. Các tùy chọn là 'cubic' và 'akima'. Mặc định là 'cubic'.
    dt (float): Bước thời gian lấy mẫu lại. Mặc định là 0,05.

    Returns:
    None
    """
    # Check if directories exist
    if not os.path.exists(skybrush_dir):
        print(f"Directory not found: {skybrush_dir}")
        return

    if not os.path.exists(processed_dir):
        print(f"Directory not found: {processed_dir}")
        return

    # Process all csv files in the skybrush directory
    for filename in os.listdir(skybrush_dir):
        if filename.endswith(".csv"):

            try:
                # Load csv data
                filepath = os.path.join(skybrush_dir, filename)
                df = pd.read_csv(filepath)

                # Resample to 0.05 seconds (20Hz) using cubic spline interpolation
                x = df['Time [msec]'] / 1000  # convert msec to sec

                # Multiply z-axis values by -1
                df['z [m]'] = df['z [m]'] * -1

                # Chọn phương pháp nội suy
                if method == 'cubic':
                    Interpolator = CubicSpline
                elif method == 'akima':
                    Interpolator = Akima1DInterpolator
                else:
                    print(f"Phương pháp nội suy không xác định: {method}. Sử dụng 'cubic' làm mặc định.")
                    Interpolator = CubicSpline

                cs_pos = Interpolator(x, df[['x [m]', 'y [m]', 'z [m]']])
                cs_led = Interpolator(x, df[['Red', 'Green', 'Blue']])

                # Tạo dấu thời gian và dữ liệu mới
                t_new = np.arange(0, x.iloc[-1], dt)
                pos_new = cs_pos(t_new)
                led_new = cs_led(t_new)

                # Tính vận tốc và gia tốc
                vel_new = cs_pos.derivative()(t_new)
                acc_new = cs_pos.derivative().derivative()(t_new)

                # Chuẩn bị khung dữ liệu cho dữ liệu mới
                data = {
                    'idx': np.arange(len(t_new)),
                    't': t_new,
                    'px': pos_new[:, 0],
                    'py': pos_new[:, 1],
                    'pz': pos_new[:, 2],
                    'vx': vel_new[:, 0],
                    'vy': vel_new[:, 1],
                    'vz': vel_new[:, 2],
                    'ax': acc_new[:, 0],
                    'ay': acc_new[:, 1],
                    'az': acc_new[:, 2],
                    'yaw': 0,  # placeholder
                    'mode': 70,  # placeholder
                    'ledr': led_new[:, 0],
                    'ledg': led_new[:, 1],
                    'ledb': led_new[:, 2],
                }

                df_new = pd.DataFrame(data)

                # Lưu vào thư mục đã xử lý
                new_filepath = os.path.join(processed_dir, filename)
                df_new.to_csv(new_filepath, index=False)

                print(f"Processed file saved to {new_filepath}")

            except Exception as e:
                print(f"Error processing file {filename}: {e}")
