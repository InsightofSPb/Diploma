import rosbag
bag = rosbag.Bag('src/realsense.bag')
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
   print(msg)
bag.close()