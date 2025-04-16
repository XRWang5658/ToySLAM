#!/usr/bin/env python

import os
import argparse
import rosbag
import rospy

def main():
    parser = argparse.ArgumentParser(description="Rewrite bag timestamps to match msg.header.stamp")
    parser.add_argument('input_bag', help='Path to the input bag file')
    parser.add_argument('output_bag', help='Path to the output bag file (directory will be created if missing)')
    args = parser.parse_args()

    input_bag = args.input_bag
    output_bag = args.output_bag

    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(os.path.abspath(output_bag))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print("[INFO] Created output directory: {}".format(output_dir))

    # Rewrite bag
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                new_t = msg.header.stamp
            else:
                new_t = t  # fallback if no header

            outbag.write(topic, msg, new_t)

    print("[DONE] Wrote retimed bag to: {}".format(output_bag))

if __name__ == '__main__':
    main()
