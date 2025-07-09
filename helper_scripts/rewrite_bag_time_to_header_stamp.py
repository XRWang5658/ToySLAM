#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse
import rosbag
import rospy
import datetime

# GPS时间和UTC时间之间的闰秒差。
# 截至2017年1月，这个值是18。对于大多数应用来说，这是一个足够精确的近似值。
# GPS time is ahead of UTC time.
LEAP_SECONDS = 18

def gps_to_ros_time(gps_week, gps_milliseconds):
    """
    将Novatel消息中的GPS周和周内毫秒转换为rospy.Time对象。
    
    Args:
        gps_week (int): GPS周数。
        gps_milliseconds (int): GPS周内的毫秒数。
        
    Returns:
        rospy.Time: 转换后的ROS时间对象。
    """
    # GPS的起始时间是 1980年1月6日 00:00:00 UTC
    gps_epoch = datetime.datetime(1980, 1, 6, 0, 0, 0, tzinfo=datetime.timezone.utc)
    
    # 将毫秒转换为秒
    gps_seconds = gps_milliseconds / 1000.0
    
    # 计算从GPS起始时间到现在的总时间
    time_since_epoch = datetime.timedelta(weeks=gps_week, seconds=gps_seconds)
    
    # 计算当前的GPS时间点
    current_gps_time = gps_epoch + time_since_epoch
    
    # 从GPS时间中减去闰秒，得到UTC时间
    current_utc_time = current_gps_time - datetime.timedelta(seconds=LEAP_SECONDS)
    
    # 将UTC时间转换为Unix时间戳（自1970年1月1日起的秒数）
    unix_timestamp = current_utc_time.timestamp()
    
    # 从秒数创建rospy.Time对象
    return rospy.Time.from_sec(unix_timestamp)

def main():
    parser = argparse.ArgumentParser(description="根据msg.header.stamp或GPS时间重写rosbag的时间戳。")
    parser.add_argument('input_bag', help='输入rosbag文件的路径')
    parser.add_argument('output_bag', help='输出rosbag文件的路径 (如果目录不存在，将会被创建)')
    args = parser.parse_args()

    input_bag_path = args.input_bag
    output_bag_path = args.output_bag

    # 如果输出目录不存在，则创建它
    output_dir = os.path.dirname(os.path.abspath(output_bag_path))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"[INFO] 已创建输出目录: {output_dir}")

    print(f"[INFO] 正在处理输入文件: {input_bag_path}")
    msg_count = 0
    
    # 重写rosbag
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        # 使用read_messages()读取消息，它会返回(topic, msg, t)
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            # 特殊处理Novatel INSPVAX消息
            if msg._type == 'novatel_msgs/INSPVAX':
                # 确保消息头和所需字段存在
                if hasattr(msg, 'header') and hasattr(msg.header, 'gps_week') and hasattr(msg.header, 'gps_week_seconds'):
                    # 从GPS时间计算新的时间戳
                    new_t = gps_to_ros_time(msg.header.gps_week, msg.header.gps_week_seconds)
                else:
                    new_t = t # 如果字段缺失，则回退到原始包时间
            
            # 处理带有标准ROS头的时间戳
            elif hasattr(msg, 'header') and hasattr(msg.header, 'stamp') and msg.header.stamp.secs != 0:
                new_t = msg.header.stamp
            
            # 对于其他所有消息，使用原始的包时间
            else:
                new_t = t

            outbag.write(topic, msg, new_t)
            msg_count += 1
            if msg_count % 1000 == 0:
                print(f"  已处理 {msg_count} 条消息...")

    print(f"[DONE] 处理完成！总共写入 {msg_count} 条消息到: {output_bag_path}")

if __name__ == '__main__':
    main()
