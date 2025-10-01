#!/usr/bin/env python3

import os
import csv
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix
from pyproj import Proj, transform, Transformer

def get_ros2_msg_type(bag_reader, topic_name):
    """Reads bag metadata to find the message type for a given topic."""
    for topic_metadata in bag_reader.get_all_topics_and_types():
        if topic_metadata.name == topic_name:
            return topic_metadata.type
    return None

def main():
    # --- 경로 직접 지정 ---
    bag_path = "/home/hannibal/Mandol_ws/rosbag/gps_bag_9_16/T_parallel_1"
    
    # bag_path에서 파일 이름 추출하여 CSV 경로 생성
    bag_filename = os.path.basename(bag_path)
    csv_filename = os.path.splitext(bag_filename)[0] + ".csv"
    csv_path = os.path.join("/home/yoo/workspace/Mandol_ws/data/processed", csv_filename)
    # ---------------------

    topic_name = "/f9r/fix"

    print(f"Input bag path: {bag_path}")
    print(f"Output CSV path: {csv_path}")

    try:
        # Bag 파일 리더 설정
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # 토픽 필터 설정
        storage_filter = StorageFilter(topics=[topic_name])
        reader.set_filter(storage_filter)

        # 메시지 타입 가져오기
        msg_type_name = get_ros2_msg_type(reader, topic_name)
        if not msg_type_name:
            print(f"Error: Topic '{topic_name}' not found in bag file.")
            return
        
        # 동적으로 메시지 클래스 가져오기 (여기서는 NavSatFix로 고정)
        msg_type = NavSatFix

        # CSV 파일 쓰기 설정
        with open(csv_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['X(E/m)', 'Y(N/m)'])
            
            message_count = 0
            first_msg = True
            transformer = None

            while reader.has_next():
                (topic, data, t) = reader.read_next()
                msg = deserialize_message(data, msg_type)

                # 첫 메시지로 UTM 존 설정
                if first_msg:
                    utm_zone = int((msg.longitude + 180) / 6) + 1
                    # pyproj Transformer 생성 (WGS84 to UTM)
                    proj_string = f"+proj=utm +zone={utm_zone} +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
                    transformer = Transformer.from_crs("EPSG:4326", proj_string, always_xy=True)
                    first_msg = False

                # UTM으로 변환
                easting, northing = transformer.transform(msg.longitude, msg.latitude)

                # CSV에 쓰기 (최대 정밀도)
                csv_writer.writerow([f"{easting:.15f}", f"{northing:.15f}"])
                message_count += 1

        print(f"Successfully processed {message_count} messages and saved to {csv_path}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()