#!/usr/bin/env python

'''
Counts tag ids in a rosbag
'''


import rosbag
from rfid_node.msg import TagReading



# ...................................................................................................
# Main function.
if __name__ == '__main__':
    ## params

    # this was a nice clean rosbag
    #bagFolder = '/home/manolofc/ILIAD_DATASETS/HRSI_situation_QTC_rosbags/ot/hotl/'
    #bagFile = '10.bag'

    # this has a couple of missing points due to incomplete tfs
    
    bagFolder = '/home/manuel/workspace/nbs_ws/src/RFID/rfid_grid_map/test/fdg_tests3/'    
    bagFile = 'TiagoRfidFDG_1.bag'

    # topic names in bag file
    tag_topic = '/lastTag'

    
    tagIDSet=set()
        
    # Process bag  ........................................................

    print("..................................")
    print("Processing bag: " + bagFile)
    bag = rosbag.Bag(bagFolder+bagFile)

    # Process bag ..........................................................
    for topic, msg, t in bag.read_messages():
        if topic == tag_topic:
            tag_id = msg.ID
            if not (tag_id in tagIDSet):
                tagIDSet.add(tag_id)
                print("New Tag detected [" + tag_id + "] at time [" + str(t.to_sec())+ "]")

    # close bag
    bag.close()

    