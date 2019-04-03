#!/usr/bin/env python

'''
Truly ofline rosbag parser!
Bag contains: tf topic with robot tf and rfid tags tf
              rfid readings
              laser readings (in case you want to use AMCL to improve localization)
'''


import rosbag
from tf_bag import BagTfTransformer

import tf
import pandas as pd
from rfid_node.msg import TagReading


def getRelativeXYYaw(bag_tf, orig_frame, dest_frame, t):
    translation, quaternion = bag_tf.lookupTransform(orig_frame, dest_frame, t)
    (rel_x, rel_y, rel_yaw) = getXYYaw(translation, quaternion)
    return (rel_x, rel_y, rel_yaw)

def getXYYaw(translat,rotat):
    rel_x = translat[0]
    rel_y = translat[1]
    (rel_rol, rel_pitch, rel_yaw) = tf.transformations.euler_from_quaternion(rotat)
    return (rel_x, rel_y, rel_yaw)

# Main function.
if __name__ == '__main__':
    folder = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/'
    saveFile = folder  + '20dB-Linda-FAB-LAB-V3.csv'
    bagFile = folder  + '2000-Linda-FAB-LAB.bag'

    rfid_reading_topic = '/lastTag'
    robot_frame = 'base_footprint'
    map_frame = "map"

    tagTFPrefix = "rfid/tags/"

    labels = ['Time','ID','rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg','robot_x_m', 'robot_y_m', 'robot_yaw_rad']
    dataEntries = []

    bag = rosbag.Bag(bagFile)
    bag_transformer = BagTfTransformer(bag)

    # main loop
    for topic, msg, t in bag.read_messages():
        if topic == rfid_reading_topic:
            tag_time = str(msg.timestamp.to_nsec())
            tid = str(msg.ID)
            rssi_db = str(msg.rssi)
            phase_deg = str(msg.phase)
            freq_khz = str(msg.frequency)

            (rob_x, rob_y, rob_yaw) = getRelativeXYYaw(bag_transformer,map_frame, robot_frame, t)

            tag_frame = 'rfid/tags/'+tid
            (rel_x, rel_y, rel_yaw)  = getRelativeXYYaw(bag_transformer,robot_frame, tag_frame, t)

            # entry order and elements MUST MATCH LABELS!!!!!
            entry = (tag_time, tid, rel_x, rel_y, rel_yaw, freq_khz, rssi_db, phase_deg, rob_x, rob_y, rob_yaw)
            dataEntries.append(entry)

    # save and close
    bag.close()

    df = pd.DataFrame.from_records(dataEntries, columns=labels)
    print("Saving data to csv")
    df.to_csv(saveFile, index=False)

