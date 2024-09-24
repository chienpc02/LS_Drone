import pandas as pd
import os


def update_config_file(skybrush_dir, config_file):
    """
        Chức năng cập nhật cột 'x' và 'y' của tệp cấu hình với vị trí ban đầu của từng drone.

        Tham số:
        skybrush_dir (str): Thư mục chứa các file drone.
        config_file(str): Đường dẫn của file config cần cập nhật.

    Returns:
    None
    """
    # Check if directories exist
    if not os.path.exists(skybrush_dir):
        print(f"Không tìm thấy thư mục: {skybrush_dir}")
        return

    # Load the config file
    config_df = pd.read_csv(config_file)

    # Process all csv files in the skybrush directory
    for filename in os.listdir(skybrush_dir):
        if filename.endswith(".csv"):

            try:
                # Load csv data
                filepath = os.path.join(skybrush_dir, filename)
                df = pd.read_csv(filepath)

                # Get the initial position
                initial_x = df.loc[0, 'x [m]']
                initial_y = df.loc[0, 'y [m]']

                # Get the drone ID
                drone_id = int(filename.replace('Drone', '').replace('.csv', ''))

                # Update the config file
                config_df.loc[config_df['pos_id'] == drone_id, 'x'] = initial_x
                config_df.loc[config_df['pos_id'] == drone_id, 'y'] = initial_y

            except Exception as e:
                print(f"Error processing file {filename}: {e}")

    # Save the updated config file
    config_df.to_csv(config_file, index=False)
    print(f"Config file updated: {config_file}")
